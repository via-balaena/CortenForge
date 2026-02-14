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

### 19. `<frame>` Element Parsing + Body/Frame `childclass`
**Status:** Complete | **Effort:** M | **Prerequisites:** #18 (compiler element)

#### Current State

No `MjcfFrame` type, no `b"frame"` match in parser. Silently dropped by catch-all
`skip_element()`. Models using `<frame>` for coordinate system convenience (common
in complex humanoid MJCF files) silently lose frame transformations.

Additionally, `childclass` is not parsed on `<body>` or `<frame>` elements. The
default class resolver (`DefaultResolver`) already supports class lookup by name,
but no mechanism exists to inherit a default class from parent bodies or frames.
`MjcfBody` has no `childclass` field; `MjcfSite` has no `class` field.

#### Objective

Parse `<frame>` elements and expand their coordinate transformations into child
elements during model building. Parse and apply `childclass` on both `<body>` and
`<frame>` elements.

#### Specification

##### Part A — Types

**A1. `MjcfFrame` struct** (in `types.rs`)

A coordinate frame transformation. All children have their pos/quat interpreted
relative to the frame, not the parent body. Frames disappear during model building.

Fields:
- `name: Option<String>` — optional, not preserved in compiled model
- `pos: Vector3<f64>` — position offset, default `(0,0,0)`
- `quat: Vector4<f64>` — orientation quaternion (w,x,y,z), default `(1,0,0,0)`
- `axisangle: Option<Vector4<f64>>` — alternative orientation
- `euler: Option<Vector3<f64>>` — alternative orientation
- `xyaxes: Option<[f64; 6]>` — alternative orientation (x-axis 3 floats, y-axis 3 floats)
- `zaxis: Option<Vector3<f64>>` — alternative orientation (minimal rotation from Z)
- `childclass: Option<String>` — default class for children
- `bodies: Vec<MjcfBody>` — child bodies
- `geoms: Vec<MjcfGeom>` — child geoms
- `sites: Vec<MjcfSite>` — child sites
- `frames: Vec<MjcfFrame>` — nested child frames

Camera and light children are deferred until `MjcfCamera`/`MjcfLight` types exist
(item #49). Cameras and lights inside `<frame>` emit `tracing::warn!` and are skipped.

Allowed children: `<body>`, `<geom>`, `<site>`, `<camera>`, `<light>`, `<frame>`.
**NOT** allowed: `<joint>`, `<freejoint>`, `<inertial>` — these belong directly on
bodies. Parser returns `MjcfError::InvalidElement` if encountered.

**A2. `MjcfBody` additions** (in `types.rs`)

Add two fields:
- `frames: Vec<MjcfFrame>` — frames within this body, default empty
- `childclass: Option<String>` — default class for child elements, default `None`

**A3. `MjcfGeom` orientation additions** (in `types.rs`)

Add fields matching MuJoCo's supported geom orientations:
- `axisangle: Option<Vector4<f64>>`
- `xyaxes: Option<[f64; 6]>`
- `zaxis: Option<Vector3<f64>>`

These are needed for completeness (MuJoCo supports them on geoms) and because
frame expansion resolves child orientations to quaternions — the geom type must
be able to represent all formats before resolution.

**A4. `MjcfSite` orientation + class additions** (in `types.rs`)

Add fields:
- `euler: Option<Vector3<f64>>`
- `axisangle: Option<Vector4<f64>>`
- `xyaxes: Option<[f64; 6]>`
- `zaxis: Option<Vector3<f64>>`
- `class: Option<String>`

The `class` field is required for childclass to work (default resolution uses it).
The orientation fields partially address item #21 and are needed because sites
inside frames may use any orientation format.

**A5. `InvalidElement` error variant** (in `error.rs`)

Add: `InvalidElement(String)` — an element appeared in an invalid location
(e.g., `<joint>` inside `<frame>`).

##### Part B — Parser Changes

**B1. `parse_frame()` and `parse_frame_attrs()`** (in `parser.rs`)

Follow the exact same pattern as `parse_body()`/`parse_body_attrs()`:

`parse_frame_attrs()` extracts: `name`, `pos`, `quat`, `euler`, `axisangle`,
`xyaxes`, `zaxis`, `childclass`.

`parse_frame()` dispatches child elements:
- `b"body"` → `parse_body()` → `frame.bodies.push()`
- `b"geom"` → `parse_geom()`/`parse_geom_attrs()` → `frame.geoms.push()`
- `b"site"` → `parse_site()`/`parse_site_attrs()` → `frame.sites.push()`
- `b"frame"` → `parse_frame()` (recursive) → `frame.frames.push()`
- `b"camera"` | `b"light"` → `tracing::warn!` + `skip_element()`
- `b"joint"` | `b"freejoint"` | `b"inertial"` → `Err(MjcfError::InvalidElement(...))`
- `_` → `skip_element()`

Handle both `Event::Start` and `Event::Empty` variants (empty frame = valid no-op).

**B2. Modify `parse_body()` and `parse_worldbody()`** (in `parser.rs`)

Add `b"frame"` match arms in both `Event::Start` and `Event::Empty` branches:
```
b"frame" => {
    let frame = parse_frame(reader, e)?;  // or parse_frame_attrs(e) for Empty
    body.frames.push(frame);
}
```

**B3. Parse `childclass` on bodies** (in `parser.rs`)

In `parse_body_attrs()`, add: `body.childclass = get_attribute_opt(e, "childclass");`

**B4. Parse additional geom orientations** (in `parser.rs`)

In `parse_geom_attrs()`, add parsing for `axisangle`, `xyaxes`, `zaxis`.

**B5. Parse additional site orientations + class** (in `parser.rs`)

In `parse_site_attrs()`, add parsing for `euler`, `axisangle`, `xyaxes`, `zaxis`,
and `class`.

##### Part C — Model Builder: Frame Expansion

Frame expansion is a pre-processing step in `model_from_mjcf()`. It runs before
`discardvisual`/`fusestatic` (matching MuJoCo's order) and before `ModelBuilder`
processes bodies.

**C1. `resolve_orientation()` — unified orientation resolution** (in `model_builder.rs`)

A single function replacing the duplicated inline logic in body, geom, and site
processing. Priority when multiple alternatives are present (MuJoCo errors on
multiple; we silently use the highest-priority one for robustness):
`euler` > `axisangle` > `xyaxes` > `zaxis` > `quat`.

```
fn resolve_orientation(
    quat: Vector4<f64>,
    euler: Option<Vector3<f64>>,
    axisangle: Option<Vector4<f64>>,
    xyaxes: Option<[f64; 6]>,
    zaxis: Option<Vector3<f64>>,
    compiler: &MjcfCompiler,
) -> UnitQuaternion<f64>
```

- **euler**: apply `compiler.angle` conversion (degree→radian if needed), then
  `euler_seq_to_quat(euler_rad, &compiler.eulerseq)`.
- **axisangle**: apply `compiler.angle` conversion to the angle component (4th
  element), then `UnitQuaternion::from_axis_angle()`.
- **xyaxes**: `[x0,x1,x2, y0,y1,y2]`. Algorithm (matches MuJoCo's `ResolveOrientation`):
  1. Normalize x-axis: `x = normalize(xyaxes[0..3])`. Error if norm < eps.
  2. Gram-Schmidt orthogonalize y against x: `y -= x * dot(x, y)`.
  3. Normalize y. Error if norm < eps (x and y were parallel).
  4. Compute z = cross(x, y) (already unit-length since x, y are orthonormal).
  5. Build rotation matrix R from columns [x, y, z] and convert to quaternion
     via `nalgebra::UnitQuaternion::from_rotation_matrix()`.
  MuJoCo skips normalizing y and z, relying on post-hoc quaternion normalization
  in `frame2quat`. We normalize to ensure a valid rotation matrix for nalgebra.
  For orthogonal inputs the result is identical; for non-orthogonal inputs our
  behavior is more robust (MuJoCo's produces a slightly non-unit quaternion before
  normalization, but the normalized result is the same).
- **zaxis**: compute minimal rotation from default Z-axis `(0,0,1)` to given
  direction. Algorithm (matches MuJoCo's `mjuu_z2quat`):
  1. Normalize the input vector. Error if norm < eps.
  2. Compute `axis = cross((0,0,1), zaxis_normalized)`.
  3. Compute `s = norm(axis)`.
  4. If `s < 1e-10` (parallel or anti-parallel): set `axis = (1,0,0)`.
     Else: normalize `axis` in place (`axis /= s`).
  5. Compute `angle = atan2(s, zaxis_normalized.z)`.
  6. Build quaternion: `w = cos(angle/2)`, `xyz = axis * sin(angle/2)`.
  Result: parallel → identity; anti-parallel → 180deg about X; general → minimal
  rotation. The `atan2` handles both degenerate cases correctly.
- **quat** (fallback): `quat_from_wxyz(quat)`.

After adding this function, refactor the existing orientation resolution in
`process_body_with_world_frame()` (lines 984–1007), `process_geom()` (lines
1408–1424), and `process_site()` to call `resolve_orientation()`. This eliminates
three copies of the same logic.

**C2. `frame_accum_child()` — core SE(3) composition** (in `model_builder.rs`)

Direct translation of MuJoCo's `mjuu_frameaccumChild`:

```
fn frame_accum_child(
    frame_pos: &Vector3<f64>,
    frame_quat: &UnitQuaternion<f64>,
    child_pos: &mut Vector3<f64>,
    child_quat: &mut UnitQuaternion<f64>,
)
```

Math:
- `child_pos_new = frame_pos + frame_quat.transform_vector(child_pos_old)`
- `child_quat_new = frame_quat * child_quat_old`

**C3. `expand_frames()` — recursive frame expansion** (in `model_builder.rs`)

```
fn expand_frames(
    body: &mut MjcfBody,
    compiler: &MjcfCompiler,
    parent_childclass: Option<&str>,
)
```

Algorithm:
1. Determine effective childclass for this body's children:
   `effective = body.childclass.as_deref().or(parent_childclass)`.
2. Take `body.frames` via `std::mem::take()`.
3. For each frame, call `expand_single_frame(body, frame, &zero_pos, &identity_quat,
   compiler, effective)`.
4. Recursively call `expand_frames(child, compiler, effective)` on each child body,
   passing this body's effective childclass as the child's `parent_childclass`.

**C4. `expand_single_frame()` — single frame expansion** (in `model_builder.rs`)

```
fn expand_single_frame(
    body: &mut MjcfBody,
    frame: &MjcfFrame,
    accumulated_pos: &Vector3<f64>,
    accumulated_quat: &UnitQuaternion<f64>,
    compiler: &MjcfCompiler,
    parent_childclass: Option<&str>,
)
```

Algorithm:
1. Resolve frame's own orientation via `resolve_orientation()`.
2. Compose with accumulated parent-frame transform via `frame_accum_child()`:
   `composed = accumulated ∘ frame_local`.
3. Determine effective childclass: `frame.childclass` overrides `parent_childclass`.
4. **For each child geom** (clone, transform, push onto body):
   - If geom has no explicit `class`, set `class = effective_childclass`.
   - **`fromto` geoms**: transform both endpoints through the composed frame.
     `from_new = composed_pos + composed_quat * from_old` (same for `to`).
     Leave `pos`/`quat` untouched — `compute_fromto_pose()` will derive them later.
   - **Non-`fromto` geoms**: resolve geom's orientation via `resolve_orientation()`,
     then compose with frame via `frame_accum_child()`. Store result as `quat`,
     clear `euler`/`axisangle`/`xyaxes`/`zaxis` (already resolved).
5. **For each child site** (clone, transform, push onto body):
   - If site has no explicit `class`, set `class = effective_childclass`.
   - Resolve site orientation, compose with frame, store as `quat`, clear alternatives.
6. **For each child body** (clone, transform, push onto body.children):
   - If body has no own `childclass`, inherit `effective_childclass`.
   - Resolve body orientation, compose with frame, store as `quat`, clear `euler`/`axisangle`.
7. **For each nested frame**: recurse with the composed transform and effective childclass.

**C5. Wire into `model_from_mjcf()`** (in `model_builder.rs`)

Insert before `discardvisual`/`fusestatic` (matching MuJoCo's expansion order):

```rust
// Expand <frame> elements: compose transforms into children, lift onto parent bodies
expand_frames(&mut mjcf.worldbody, &mjcf.compiler, None);
```

**C6. Thread `inherited_childclass` through `process_body_with_world_frame()`**

Add parameter `inherited_childclass: Option<&str>` to `process_body_with_world_frame()`.

For each element type, apply childclass if the element has no explicit `class`:
- Geoms: before `self.resolver.apply_to_geom()`, set `g.class = effective_childclass`
  if `g.class.is_none()`.
- Joints: same pattern with `self.resolver.apply_to_joint()`.
- Sites: same pattern with `self.resolver.apply_to_site()`.

Pass `effective_childclass` to recursive calls for child bodies.

Update all call sites of `process_body_with_world_frame()` to include the new parameter.
Initial call from `model_from_mjcf()` passes `mjcf.worldbody.childclass.as_deref()`.

##### Part D — Default Resolver Fix

**D1. Fix `apply_to_site()` in `defaults.rs`**

Change `self.site_defaults(None)` → `self.site_defaults(site.class.as_deref())`.
This is a prerequisite for childclass to work on sites.

#### Acceptance Criteria

**Frame position:**
1. `<frame pos="1 0 0"><geom pos="0 0 0" .../></frame>` → geom at body-relative
   `(1, 0, 0)`.

**Frame rotation:**
2. `<frame euler="0 0 90">` (with `angle="degree"`) `<geom pos="1 0 0" .../></frame>`
   → geom at body-relative `(0, 1, 0)` (90deg Z rotation maps x to y).

**Frame position + rotation:**
3. `<frame pos="1 0 0" euler="0 0 90">` (degrees) `<geom pos="1 0 0" .../></frame>`
   → geom at `(1, 1, 0)`. frame_pos + frame_rot * geom_pos = (1,0,0) + (0,1,0).

**Nested frames:**
4. `<frame pos="1 0 0"><frame pos="0 1 0"><geom .../></frame></frame>` → geom at
   `(1, 1, 0)`.

**3-deep nested frames:**
5. `<frame pos="1 0 0"><frame pos="0 1 0"><frame pos="0 0 1"><geom .../></frame>
   </frame></frame>` → geom at `(1, 1, 1)`.

**Frame wrapping body:**
6. `<frame pos="2 0 0"><body pos="1 0 0">...</body></frame>` → body_pos = `(3, 0, 0)`.

**Frame with `fromto` geom:**
7. `<frame pos="1 0 0"><geom type="capsule" fromto="0 0 0 0 0 1" .../></frame>` →
   fromto endpoints become `(1,0,0)-(1,0,1)`, geom pos = midpoint `(1, 0, 0.5)`.

**Frame with `fromto` geom + rotation:**
8. `<frame euler="0 90 0">` (degrees, Y rotation) `<geom type="capsule"
   fromto="0 0 0 0 0 1" .../></frame>` → 90deg Y rotation maps Z to X.
   Endpoints: `(0,0,0)-(1,0,0)`. Geom pos approx `(0.5, 0, 0)`.

**Frame with site:**
9. `<frame pos="0.5 0 0"><site pos="0 0 0"/></frame>` → site at `(0.5, 0, 0)`.

**Empty frame:**
10. `<frame pos="1 0 0"/>` inside a body → model loads without error, no children affected.

**Frame at worldbody level:**
11. `<worldbody><frame pos="1 0 0"><geom .../></frame></worldbody>` → geom at `(1,0,0)`.

**Frame with only orientation (no pos):**
12. `<frame euler="0 0 90"><geom pos="1 0 0" .../></frame>` → geom at `(0, 1, 0)`.
    Frame pos defaults to `(0,0,0)`.

**Frame with only position (no orientation):**
13. `<frame pos="3 0 0"><geom pos="0 2 0" .../></frame>` → geom at `(3, 2, 0)`.
    Orientation defaults to identity.

**Frame with `xyaxes`:**
14. `<frame xyaxes="0 1 0 -1 0 0"><geom pos="1 0 0" .../></frame>` →
    xyaxes "0 1 0 -1 0 0" = 90deg Z rotation. Geom at `(0, 1, 0)`.

**Frame with `zaxis`:**
15. `<frame zaxis="1 0 0"><geom pos="0 0 1" .../></frame>` →
    zaxis "1 0 0" maps Z to X. Geom at local (0,0,1) → body-relative `(1, 0, 0)`.

**Frame with `axisangle`:**
16. `<frame axisangle="0 0 1 90">` (with `angle="degree"`) `<geom pos="1 0 0" .../></frame>`
    → 90deg about Z. Geom at `(0, 1, 0)`.

**Joint inside frame = error:**
17. `<frame><joint .../></frame>` → `MjcfError::InvalidElement`.

**Freejoint inside frame = error:**
18. `<frame><freejoint/></frame>` → `MjcfError::InvalidElement`.

**Inertial inside frame = error:**
19. `<frame><inertial .../></frame>` → `MjcfError::InvalidElement`.

**Childclass on body:**
20. `<body childclass="red"><geom .../></body>` → geom inherits class "red" defaults
    (when geom has no explicit `class`).

**Childclass override by explicit class:**
21. `<body childclass="red"><geom class="blue" .../></body>` → geom uses class "blue"
    (explicit overrides childclass).

**Childclass on frame:**
22. `<body childclass="red"><frame childclass="green"><geom .../></frame></body>` →
    geom inherits "green" (frame's childclass overrides body's).

**Childclass inheritance through body hierarchy:**
23. `<body childclass="robot"><body><joint .../></body></body>` → inner body's
    joint inherits class "robot" from parent (childclass propagates down).

**Childclass on child body overrides parent:**
24. `<body childclass="A"><body childclass="B"><geom .../></body></body>` → geom
    inherits "B" (child body's childclass overrides parent's).

**Regression: no frames, no childclass:**
25. Models without `<frame>` or `childclass` produce identical results to current behavior.

**Geom orientation composition:**
26. `<frame euler="0 0 90"><geom euler="90 0 0" .../></frame>` (degrees) → geom
    orientation = frame_quat * geom_quat. Both euler values resolved, then composed.

**Frame + angle="radian":**
27. `<compiler angle="radian"/><frame euler="0 0 1.5707963"><geom pos="1 0 0" .../></frame>`
    → geom at `(0, 1, 0)` (same as degree case, just radian input).

#### Implementation Notes

**Expansion ordering in `model_from_mjcf()`:**
Frame expansion runs BEFORE `discardvisual`/`fusestatic`, matching MuJoCo's behavior
(MuJoCo expands frames during XML parsing, before any compiler passes). This ensures
geoms/sites inside frames are visible to discardvisual (can be culled if visual-only)
and fusestatic (participate in static body fusion). After expand_frames, all frame
vecs are empty, so fusestatic doesn't need to handle them.
Order: **expand_frames** → discardvisual → fusestatic → builder.

**`fromto` edge case detail:** When a `fromto` geom is inside a frame, the frame
transform must be applied to the raw `fromto` endpoints, NOT to the derived
pos/quat. The model builder's `compute_fromto_pose()` will later derive pos/quat
from the (already-transformed) endpoints. If we instead transformed the derived
pos/quat, the capsule/cylinder axis would be wrong because `fromto` derives
orientation from the endpoint vector direction, which is frame-dependent.
The geom's `pos` and `quat` fields are irrelevant for `fromto` geoms —
`compute_fromto_pose()` overwrites them from the endpoints. During frame expansion,
we only transform the `fromto` array and leave `pos`/`quat` at their parsed values
(they will be overwritten later). Do NOT also transform `pos`/`quat`.

**Geom orientation after frame expansion:** After composing a geom's orientation
with the frame's, the result is stored as a `quat` and all alternative orientation
fields (`euler`, `axisangle`, `xyaxes`, `zaxis`) are set to `None`. This prevents
the model builder from re-resolving an already-resolved orientation.

**Bodies inside frames:** When a body is inside a frame, the frame's transform is
composed into the body's `pos`/`quat`, and the body is then appended to
`parent_body.children`. The body's own joints, geoms, and sites are NOT
transformed by the frame — they are relative to their own body, which has
already absorbed the frame transform. The body's `euler`/`axisangle` are cleared
after resolution (stored as `quat`).

**childclass is NOT applied to bodies themselves:** childclass provides a default
`class` for a body's *children* (geoms, joints, sites). It does not set the body's
own class. A body inside a frame with `childclass="X"` inherits `X` as its own
childclass (for its children), not as its own class.

**Dual-path childclass application:** Childclass is applied in two places:
(a) During frame expansion (C4): elements inside frames get their `class` field set
to the effective childclass before being lifted onto the body. (b) During model
building (C6): elements directly on bodies (not from frames) get childclass applied
before default resolution. An element that came from a frame already has its `class`
set (or was already explicit), so C6's "if class is None" check naturally skips it.
There is no double-application risk.

**Refactoring `resolve_orientation()`:** The existing inline orientation resolution
(3 copies: body at model_builder.rs:984, geom at :1408, site at :1531) should be
replaced with calls to the new `resolve_orientation()` function. This is part of
the implementation scope — not a separate refactoring item.

#### Files

- `sim/L0/mjcf/src/types.rs` — `MjcfFrame` struct; `frames`/`childclass` on
  `MjcfBody`; `axisangle`/`xyaxes`/`zaxis` on `MjcfGeom`;
  `euler`/`axisangle`/`xyaxes`/`zaxis`/`class` on `MjcfSite`
- `sim/L0/mjcf/src/parser.rs` — `parse_frame()`, `parse_frame_attrs()`;
  `b"frame"` arms in `parse_body()` and `parse_worldbody()`; `childclass` parsing
  in `parse_body_attrs()`; additional orientation parsing in `parse_geom_attrs()`
  and `parse_site_attrs()`
- `sim/L0/mjcf/src/model_builder.rs` — `resolve_orientation()`,
  `frame_accum_child()`, `expand_frames()`, `expand_single_frame()`;
  wire into `model_from_mjcf()`; thread `inherited_childclass` through
  `process_body_with_world_frame()`; refactor body/geom/site orientation resolution
- `sim/L0/mjcf/src/defaults.rs` — fix `apply_to_site()` to use `site.class`
- `sim/L0/mjcf/src/error.rs` — add `InvalidElement(String)` variant

---

### 20. `childclass` Attribute — Edge Cases & Validation
**Status:** ✅ Done | **Effort:** S | **Prerequisites:** #19 (complete)

> **Scope note:** Core `childclass` mechanism (parsing, inheritance, application
> to geoms/joints/sites, frame `childclass`, recursive propagation) was fully
> implemented in item #19 with 6 passing tests (AC20–AC25). This item hardens
> that implementation with edge-case coverage, one targeted code fix (nonexistent
> class validation), and documentation of MuJoCo-conformant behavior boundaries.

#### Current State (Post-#19)

All core functionality is working. The following is implemented and tested:

| Component | Location | Status |
|-----------|----------|--------|
| `MjcfBody.childclass` field | `types.rs:1896` | ✅ Parsed (`parser.rs:1267`) |
| `MjcfFrame.childclass` field | `types.rs:1833` | ✅ Parsed (`parser.rs:1652`) |
| Effective childclass computation | `model_builder.rs:1024` | ✅ `body.childclass.or(inherited)` |
| Geom class injection | `model_builder.rs:1028-1038` | ✅ `if g.class.is_none() → effective` |
| Joint class injection | `model_builder.rs:1118-1123` | ✅ Same pattern |
| Site class injection | `model_builder.rs:1146-1151` | ✅ Same pattern |
| Recursive propagation to children | `model_builder.rs:1163` | ✅ Passes `effective_childclass` |
| Frame expansion childclass | `model_builder.rs:3606` | ✅ `frame.childclass.or(parent)` |
| Frame→geom/site class injection | `model_builder.rs:3613-3614, 3650-3651` | ✅ |
| Frame→body childclass inheritance | `model_builder.rs:3673-3676` | ✅ |
| Childclass validation (new) | `model_builder.rs:3494-3540` | ✅ Pre-expansion validation |
| Default resolver site fix | `defaults.rs:325` | ✅ Uses `site.class.as_deref()` |

**Existing tests (6):** AC20 (basic body childclass), AC21 (explicit class overrides),
AC22 (frame childclass overrides body's), AC23 (2-level hierarchy inheritance),
AC24 (child body overrides parent), AC25 (regression without childclass).

**Remaining gaps:** Nonexistent class validation, nested default hierarchy, 3-level
deep propagation, multi-element simultaneous application, nested frames, empty body.

#### Objective

Harden `childclass` implementation with (a) validation that `childclass` references
a defined default class (MuJoCo errors on undefined classes), and (b) comprehensive
edge-case test coverage.

#### MuJoCo Authoritative Semantics

These rules govern all behavior. Every acceptance criterion traces to one or more:

- **S1 — Recursive propagation:** `childclass` on a `<body>` applies to all
  descendant elements (geoms, joints, sites) recursively, not just direct children.
  MuJoCo docs: "causing all children of this body (and all their children etc.)
  to use class X unless specified otherwise."
- **S2 — Element default, not body class:** `childclass` does NOT set the body's
  own class. It provides a default `class` for the body's child *elements* (geoms,
  joints, sites, cameras, lights).
- **S3 — Precedence chain:** explicit `class=` on element > nearest ancestor
  body/frame `childclass` > unnamed top-level default (class `""`).
- **S4 — Worldbody exclusion:** `<worldbody>` accepts NO attributes in MuJoCo's
  schema. Our parser never reads `<worldbody>` attributes (`parse_worldbody()` at
  `parser.rs:1090`), so `worldbody.childclass` is always `None` — correct by
  construction.
- **S5 — Outside kinematic tree:** `childclass` does NOT affect tendons or actuators.
  They live in `<tendon>` and `<actuator>` sections outside the body tree and have
  their own `class` attribute for explicit assignment.
- **S6 — Default hierarchy resolution:** When `childclass` references a named class
  (e.g., `"arm"`), and `"arm"` is a child of `"robot"` in the `<default>` hierarchy,
  the full inheritance chain resolves: `"arm"` inherits from `"robot"`. The
  `DefaultResolver` handles this automatically via `resolve_single()`.
- **S7 — Undefined class = error:** `childclass` referencing a class name that does
  not exist in `<default>` is an error. MuJoCo rejects this at schema validation.

#### Specification

##### Part A — Validation: Undefined `childclass` References (Code Fix)

**A1. `validate_childclass_references()` function** (in `model_builder.rs`)

Add a validation pass that runs BEFORE `expand_frames()` in `model_from_mjcf()`.
This must run before frame expansion because `expand_frames` dissolves frames
(via `std::mem::take`), losing `frame.childclass` values.

Two free functions near `expand_frames` (~line 3477):

```rust
fn validate_childclass_references(
    body: &MjcfBody,
    resolver: &DefaultResolver,
) -> std::result::Result<(), ModelConversionError>
```

Walks the body tree recursively. For each `body.childclass`, checks
`resolver.get_defaults(Some(cc))`. If `None` (class not found in resolved
defaults), returns `Err` with message naming the class and body.

Also walks `body.frames` recursively via a helper:

```rust
fn validate_frame_childclass_refs(
    frame: &MjcfFrame,
    resolver: &DefaultResolver,
) -> std::result::Result<(), ModelConversionError>
```

Same check for `frame.childclass`. Recurses into `frame.frames` (nested) and
`frame.bodies` (bodies inside frames may have their own childclass).

**A2. Wire into `model_from_mjcf()`** (at `model_builder.rs:211-216`)

Between the clone and `expand_frames`:

```rust
let mut mjcf = mjcf.clone();

// Validate childclass references before frame expansion dissolves frames
let pre_resolver = DefaultResolver::from_model(&mjcf);
validate_childclass_references(&mjcf.worldbody, &pre_resolver)?;

expand_frames(&mut mjcf.worldbody, &mjcf.compiler, None);
```

The `DefaultResolver` is constructed twice (once here for validation, once at
line 226 for the builder). This is acceptable — construction is a cheap HashMap
build from a small vec of defaults. Correctness over premature optimization.

**Key implementation detail:** `DefaultResolver::get_defaults(Some("name"))` at
`defaults.rs:72-74` looks up `"name"` in `resolved_defaults` HashMap. Returns
`None` for undefined class names. This is exactly the check needed.

##### Part B — Documentation (Doc Comments Only)

**B1.** At `model_builder.rs:252` (the `process_body` call): Add comment
explaining that `worldbody.childclass` is always `None` for parsed MJCF per
MuJoCo semantics — `<worldbody>` accepts no attributes. The programmatic API
does not enforce this; enforcement is structural via the parser.

**B2.** At `model_builder.rs:886` (in `process_worldbody_geoms_and_sites`): Add
comment noting that no childclass is applied to worldbody's own geoms/sites,
which is correct per MuJoCo — worldbody has no childclass (S4).

##### Part C — Edge-Case Tests

Eight new tests in the existing test module in `model_builder.rs`, after AC27.
All use `load_model()` and assert on compiled model fields. (AC26–AC27 are used
by item #19's frame orientation composition tests.)

**AC28 — Childclass referencing a nested default class** (S6)

Verifies that childclass resolves through the full `<default>` inheritance chain:
class `"arm"` inherits from parent class `"robot"`.

```xml
<mujoco>
  <default>
    <default class="robot">
      <geom contype="5"/>
      <default class="arm">
        <geom conaffinity="3"/>
      </default>
    </default>
  </default>
  <worldbody>
    <body name="b" childclass="arm">
      <geom type="sphere" size="0.1"/>
    </body>
  </worldbody>
</mujoco>
```

Assert: geom `contype == 5` (inherited from `"robot"` through `"arm"` chain)
AND geom `conaffinity == 3` (from `"arm"` directly).

**AC29 — Childclass applies to geom, joint, AND site simultaneously** (S1)

Verifies all three element types receive defaults from a single childclass.

```xml
<mujoco>
  <compiler angle="radian"/>
  <default>
    <default class="R">
      <geom contype="7"/>
      <joint damping="5.0"/>
      <site size="0.05"/>
    </default>
  </default>
  <worldbody>
    <body name="b" childclass="R">
      <joint name="j" type="hinge"/>
      <geom type="sphere" size="0.1"/>
      <site name="s"/>
    </body>
  </worldbody>
</mujoco>
```

Assert: geom `contype == 7`, joint `damping == 5.0`, site `size == 0.05`.

**AC30 — 3-level deep propagation without override** (S1)

Verifies childclass propagates through 3 body levels without any intermediate
body declaring its own childclass.

```xml
<mujoco>
  <compiler angle="radian"/>
  <default>
    <default class="X">
      <joint damping="3.0"/>
    </default>
  </default>
  <worldbody>
    <body name="top" childclass="X">
      <geom type="sphere" size="0.1"/>
      <body name="mid" pos="0 0 1">
        <geom type="sphere" size="0.1"/>
        <body name="bot" pos="0 0 1">
          <joint name="j" type="hinge"/>
          <geom type="sphere" size="0.1"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
```

Assert: joint `damping == 3.0` (propagated from `"top"` through `"mid"` to `"bot"`).

**AC31 — 3-level with mid-hierarchy override** (S1, S3)

Verifies that a childclass override at the middle level takes effect for all
descendants below it, not the top-level childclass.

```xml
<mujoco>
  <compiler angle="radian"/>
  <default>
    <default class="A"><joint damping="1.0"/></default>
    <default class="B"><joint damping="9.0"/></default>
  </default>
  <worldbody>
    <body name="top" childclass="A">
      <geom type="sphere" size="0.1"/>
      <body name="mid" childclass="B" pos="0 0 1">
        <geom type="sphere" size="0.1"/>
        <body name="bot" pos="0 0 1">
          <joint name="j" type="hinge"/>
          <geom type="sphere" size="0.1"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
```

Assert: joint `damping == 9.0` (from `"B"`, not `"A"`).

**AC32 — Nested frames with childclass inheritance and override** (S1)

Verifies childclass propagation through nested `<frame>` elements: inner frame
inherits outer's childclass; separate inner frame overrides with its own.

```xml
<mujoco>
  <default>
    <default class="F1"><geom contype="4"/></default>
    <default class="F2"><geom contype="8"/></default>
  </default>
  <worldbody>
    <body name="b">
      <frame childclass="F1">
        <frame>
          <geom name="g1" type="sphere" size="0.1"/>
        </frame>
        <frame childclass="F2">
          <geom name="g2" type="sphere" size="0.1"/>
        </frame>
      </frame>
    </body>
  </worldbody>
</mujoco>
```

Assert: geom at index 0 (`g1`) has `contype == 4` (inherits outer frame's `"F1"`
through inner frame with no override). Geom at index 1 (`g2`) has `contype == 8`
(inner frame overrides with `"F2"`).

**AC33 — `childclass="nonexistent"` on body produces error** (S7)

```xml
<mujoco>
  <worldbody>
    <body name="b" childclass="nonexistent">
      <geom type="sphere" size="0.1"/>
    </body>
  </worldbody>
</mujoco>
```

Assert: `load_model(...)` returns `Err`. Error message contains `"nonexistent"`.

**AC34 — Body with childclass but no child elements succeeds** (correctness)

```xml
<mujoco>
  <default>
    <default class="empty"><geom contype="5"/></default>
  </default>
  <worldbody>
    <body name="b" childclass="empty" pos="0 0 1">
    </body>
  </worldbody>
</mujoco>
```

Assert: `load_model(...)` succeeds, `model.nbody == 2`, `model.ngeom == 0`.

**AC35 — `childclass="ghost"` on frame produces error** (S7)

```xml
<mujoco>
  <worldbody>
    <body name="b">
      <frame childclass="ghost">
        <geom type="sphere" size="0.1"/>
      </frame>
    </body>
  </worldbody>
</mujoco>
```

Assert: `load_model(...)` returns `Err`. Error message contains `"ghost"`.

#### What This Item Does NOT Cover

- `childclass` on `<camera>` or `<light>` — these types are not yet parsed (item #49)
- `childclass` interaction with tendons, actuators, or equality constraints — they
  are outside the body tree (S5) and have their own `class` attributes
- Schema-level rejection of `childclass` on `<worldbody>` — correct by construction (S4)
- `range` as a defaultable joint attribute — not in `MjcfJointDefaults` (separate item)
- Programmatic API enforcement that `worldbody.childclass` must be `None`

#### Implementation Notes

**Validation ordering:** The validation must run BEFORE `expand_frames()` because
frame expansion dissolves `MjcfFrame` objects via `std::mem::take()`, permanently
losing `frame.childclass` values. After expansion, frames are empty vecs and
childclass from frames has already been applied to child elements' `class` fields.

**No impact on existing `test_nonexistent_class_no_panic`:** That test in
`default_classes.rs` tests an explicit `class="nonexistent"` on a joint element,
which is graceful fallthrough (no defaults applied). This is distinct from
`childclass="nonexistent"` on a body, which is now an error. The distinction:
explicit `class` on an element says "use these specific defaults" (graceful if
not found); `childclass` on a body says "all my children default to this class"
(error if not found, because it indicates a model authoring mistake).

**`childclass=""` edge case:** Our `DefaultResolver::get_defaults(Some(""))` returns
the unnamed root default (stored as key `""` in `resolved_defaults`), so
`childclass=""` passes validation and acts as a no-op (elements already fall back
to the root default). MuJoCo stores the root default as `"main"` internally, so
`""` would not match. This is a negligible divergence — no real MJCF model uses
`childclass=""`, and the behavioral outcome is equivalent (root default applies
either way). Acceptable as-is.

**Single file change:** All code changes and tests are in
`sim/L0/mjcf/src/model_builder.rs`. No type changes, no API changes, no new
dependencies.

#### Files

- `sim/L0/mjcf/src/model_builder.rs` — `validate_childclass_references()` +
  `validate_frame_childclass_refs()` (~35 lines); wire into `model_from_mjcf()`
  (~3 lines); 2 doc comments; 8 tests AC28–AC35 (~200 lines)

---

### 21. `<site>` euler/axisangle/xyaxes/zaxis Orientation
**Status:** ✅ Done | **Effort:** S | **Prerequisites:** None

#### Current State

Site orientation was fully implemented as part of item #19. All production code is
in place and working:

| Component | Location | Status |
|-----------|----------|--------|
| `MjcfSite` with all 5 orientation fields | `types.rs:1296-1319` | ✅ Done |
| `parse_site_attrs()` parses euler/axisangle/xyaxes/zaxis/quat | `parser.rs:1527-1564` | ✅ Done |
| `process_site()` calls `resolve_orientation()` | `model_builder.rs:1554-1561` | ✅ Done |
| `resolve_orientation()` unified function (body/geom/site/frame) | `model_builder.rs:3388-3468` | ✅ Done |
| Frame expansion composes site orientation | `model_builder.rs:3647-3666` | ✅ Done |
| `apply_to_site()` uses `site.class` correctly | `defaults.rs:322-349` | ✅ Done |

**Test coverage gap:** Zero dedicated tests verify the resolved quaternion for any
site orientation specifier. The only existing test that touches site orientation
(`test_fusestatic_site_with_axisangle`, line 8173) validates position only, not
the orientation quaternion.

#### Objective

Add comprehensive acceptance tests that verify all 5 orientation specifiers on
`<site>` elements produce the correct `site_quat` values in the compiled model.
Each test validates the quaternion value, not merely that the model loads.

**No production code changes.** This item is test-only.

#### Specification

##### Orientation resolution semantics (already implemented)

All orientation specifiers on sites use `resolve_orientation()` — the same unified
function shared by body, geom, site, and frame elements. The semantics:

- **Priority** (when multiple specifiers present — MuJoCo errors on this; we
  silently use highest priority for robustness):
  `euler` > `axisangle` > `xyaxes` > `zaxis` > `quat`.
  This matches `resolve_orientation()` at `model_builder.rs:3385-3387`.

- **Compiler angle unit** (`compiler.angle`): applies to `euler` (all 3 components)
  and `axisangle` (4th element only — axis stays untouched). Default: `Degree`.

- **Euler sequence** (`compiler.eulerseq`): configures rotation order for `euler`.
  Default: `"xyz"` (intrinsic XYZ). Lowercase = intrinsic (post-multiply),
  uppercase = extrinsic (pre-multiply). Matches MuJoCo's `mju_euler2Quat`.

- **xyaxes**: Gram-Schmidt orthogonalization. X-axis normalized, Y-axis
  orthogonalized against X then normalized, Z = cross(X, Y). Matches MuJoCo's
  `ResolveOrientation`.

- **zaxis**: Minimal rotation from default Z `(0,0,1)` to target direction.
  Matches MuJoCo's `mjuu_z2quat`. Parallel → identity; anti-parallel → 180°
  about X; general → `atan2`-based rotation.

- **Defaults**: `MjcfSiteDefaults` covers `site_type`, `size`, `rgba` only —
  orientation is NOT part of site defaults (correct per MuJoCo). Orientation
  always comes from the element itself.

#### Acceptance Criteria

All tests go in `sim/L0/mjcf/src/model_builder.rs` in the existing `#[cfg(test)]
mod tests` block, grouped after `test_fusestatic_site_with_axisangle` (~line 8198).
Each test uses `load_model()` and asserts on `model.site_quat[0]`.

Tolerance: `1e-10` for exact single-axis cases, `1e-6` for composed/numerical
cases (matching existing conventions: line 6257 uses `1e-10`, line 7517 uses
`1e-6`).

##### Core orientation formats

**AC1 — `test_site_euler_orientation_degrees`**

```xml
<mujoco>
  <worldbody>
    <body name="b">
      <geom type="sphere" size="0.1" mass="1.0"/>
      <site name="s" euler="90 0 0"/>
    </body>
  </worldbody>
</mujoco>
```

Default compiler: `angle="degree"`, `eulerseq="xyz"`. Intrinsic X rotation 90°.
Expected: `UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI/2)`
= (w=0.7071068, x=0.7071068, y=0, z=0).

**AC2 — `test_site_euler_orientation_radians`**

```xml
<mujoco>
  <compiler angle="radian"/>
  <worldbody>
    <body name="b">
      <geom type="sphere" size="0.1" mass="1.0"/>
      <site name="s" euler="{FRAC_PI_2} 0 0"/>
    </body>
  </worldbody>
</mujoco>
```

Same rotation as AC1 but in radians. Expected: same quaternion.
Use `format!` to inject `FRAC_PI_2` (pattern from `test_body_axisangle_radian`,
line 6264).

**AC3 — `test_site_euler_orientation_zyx_eulerseq`**

```xml
<mujoco>
  <compiler angle="radian" eulerseq="ZYX"/>
  <worldbody>
    <body name="b">
      <geom type="sphere" size="0.1" mass="1.0"/>
      <site name="s" euler="0.3 0.2 0.1"/>
    </body>
  </worldbody>
</mujoco>
```

Expected: `euler_seq_to_quat(Vector3::new(0.3, 0.2, 0.1), "ZYX")`.
Follows pattern from `test_euler_seq_zyx_body_orientation` (line 6290).

**AC4 — `test_site_axisangle_orientation_degrees`**

```xml
<mujoco>
  <worldbody>
    <body name="b">
      <geom type="sphere" size="0.1" mass="1.0"/>
      <site name="s" axisangle="0 0 1 90"/>
    </body>
  </worldbody>
</mujoco>
```

90° about Z. Expected: `UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI/2)`
= (w=0.7071068, x=0, y=0, z=0.7071068).

**AC5 — `test_site_axisangle_orientation_radians`**

```xml
<mujoco>
  <compiler angle="radian"/>
  <worldbody>
    <body name="b">
      <geom type="sphere" size="0.1" mass="1.0"/>
      <site name="s" axisangle="0 1 0 {FRAC_PI_2}"/>
    </body>
  </worldbody>
</mujoco>
```

PI/2 about Y. Expected: `UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI/2)`
= (w=0.7071068, x=0, y=0.7071068, z=0).

##### xyaxes

**AC6 — `test_site_xyaxes_orientation_orthogonal`**

```xml
<mujoco>
  <worldbody>
    <body name="b">
      <geom type="sphere" size="0.1" mass="1.0"/>
      <site name="s" xyaxes="0 1 0 -1 0 0"/>
    </body>
  </worldbody>
</mujoco>
```

X→Y, Y→-X. Rotation matrix columns: `[(0,1,0), (-1,0,0), (0,0,1)]` = 90° about Z.
Expected: `UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI/2)`.

**AC7 — `test_site_xyaxes_orientation_gram_schmidt`**

```xml
<mujoco>
  <worldbody>
    <body name="b">
      <geom type="sphere" size="0.1" mass="1.0"/>
      <site name="s" xyaxes="1 1 0 0 1 0"/>
    </body>
  </worldbody>
</mujoco>
```

Non-orthogonal inputs. Gram-Schmidt:
1. x = normalize(1,1,0) = (1/√2, 1/√2, 0)
2. y = (0,1,0) - x·dot(x, (0,1,0)) = (0,1,0) - (1/√2, 1/√2, 0)·(1/√2) = (-0.5, 0.5, 0)
3. y_norm = normalize(-0.5, 0.5, 0) = (-1/√2, 1/√2, 0)
4. z = cross(x, y) = (0, 0, 1)
5. Result: 45° rotation about Z.
Expected: `UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI/4)`
= (w=0.9238795, x=0, y=0, z=0.3826834).

##### zaxis

**AC8 — `test_site_zaxis_orientation_general`**

```xml
<mujoco>
  <worldbody>
    <body name="b">
      <geom type="sphere" size="0.1" mass="1.0"/>
      <site name="s" zaxis="1 0 0"/>
    </body>
  </worldbody>
</mujoco>
```

Minimal rotation from Z=(0,0,1) to X=(1,0,0). `mjuu_z2quat` derivation:
axis = cross((0,0,1), (1,0,0)) = (0,1,0), s=1, angle = atan2(1, 0) = PI/2.
Expected: `UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI/2)`.

**AC9 — `test_site_zaxis_orientation_parallel_identity`**

```xml
<mujoco>
  <worldbody>
    <body name="b">
      <geom type="sphere" size="0.1" mass="1.0"/>
      <site name="s" zaxis="0 0 1"/>
    </body>
  </worldbody>
</mujoco>
```

Target = default Z. axis = (0,0,0), s=0 < 1e-10, fallback axis = (1,0,0),
angle = atan2(0, 1) = 0. Expected: `UnitQuaternion::identity()`.

**AC10 — `test_site_zaxis_orientation_antiparallel`**

```xml
<mujoco>
  <worldbody>
    <body name="b">
      <geom type="sphere" size="0.1" mass="1.0"/>
      <site name="s" zaxis="0 0 -1"/>
    </body>
  </worldbody>
</mujoco>
```

Target = -Z (anti-parallel). axis = cross((0,0,1), (0,0,-1)) = (0,0,0),
s=0 < 1e-10, fallback axis = (1,0,0), angle = atan2(0, -1) = PI.
Expected: `UnitQuaternion::from_axis_angle(&Vector3::x_axis(), PI)`
≈ (w≈0, x=1, y=0, z=0). Tolerance `1e-10`.

##### Regression + interactions

**AC11 — `test_site_quat_orientation_regression`**

```xml
<mujoco>
  <worldbody>
    <body name="b">
      <geom type="sphere" size="0.1" mass="1.0"/>
      <site name="s" quat="0.7071068 0 0.7071068 0"/>
    </body>
  </worldbody>
</mujoco>
```

Direct quaternion (wxyz format): 90° about Y. Regression for pre-#19 behavior.
Expected: `UnitQuaternion::from_axis_angle(&Vector3::y_axis(), PI/2)`.

**AC12 — `test_site_orientation_with_default_class`**

```xml
<mujoco>
  <default>
    <default class="sensor_site">
      <site type="cylinder" size="0.02 0.01"/>
    </default>
  </default>
  <worldbody>
    <body name="b">
      <geom type="sphere" size="0.1" mass="1.0"/>
      <site name="s" class="sensor_site" euler="0 0 90"/>
    </body>
  </worldbody>
</mujoco>
```

Defaults provide `type` + `size`; element provides `euler`. Since
`MjcfSiteDefaults` does NOT include orientation, defaults must not interfere.
Assert: `site_type[0] == GeomType::Cylinder` (from default) AND `site_quat[0]`
matches 90° Z rotation (from element).

**AC13 — `test_site_orientation_priority_euler_over_quat`**

```xml
<mujoco>
  <worldbody>
    <body name="b">
      <geom type="sphere" size="0.1" mass="1.0"/>
      <site name="s" euler="90 0 0" quat="1 0 0 0"/>
    </body>
  </worldbody>
</mujoco>
```

Both `euler` and `quat` specified. Per priority chain, euler wins. `quat` is
identity; `euler` is 90° X. Expected: 90° X rotation, NOT identity.
(MuJoCo would error on this; our intentional divergence uses highest priority.)

##### Edge cases

**AC14 — `test_site_orientation_in_frame`**

```xml
<mujoco>
  <worldbody>
    <body name="b">
      <frame euler="0 0 90">
        <site name="s" euler="90 0 0"/>
      </frame>
    </body>
  </worldbody>
</mujoco>
```

Frame 90° Z, site 90° X. Frame expansion composes: `frame_q * site_q`.
Expected: `Rz(90°) * Rx(90°)`. Tolerance `1e-6`.

**AC15 — `test_site_axisangle_non_unit_axis`**

```xml
<mujoco>
  <worldbody>
    <body name="b">
      <geom type="sphere" size="0.1" mass="1.0"/>
      <site name="s" axisangle="0 0 3 90"/>
    </body>
  </worldbody>
</mujoco>
```

Non-unit axis (0,0,3). `Unit::new_normalize(axis)` normalizes to (0,0,1).
Expected: same as `axisangle="0 0 1 90"` = 90° about Z.

**AC16 — `test_site_zaxis_non_unit_direction`**

```xml
<mujoco>
  <worldbody>
    <body name="b">
      <geom type="sphere" size="0.1" mass="1.0"/>
      <site name="s" zaxis="0 0 5"/>
    </body>
  </worldbody>
</mujoco>
```

Non-unit direction (0,0,5), parallel to default Z after normalization.
Expected: `UnitQuaternion::identity()`.

#### Implementation Notes

1. **Test-only item.** All production code (types, parsing, model building,
   defaults, frame expansion, fusestatic) is complete and correct. This item
   adds 16 acceptance tests and zero production code changes.

2. **All tests use `load_model()` + assert on `model.site_quat[0]`.** Sites are
   indexed sequentially; each test creates exactly one site at index 0.

3. **Tolerance conventions:** `1e-10` for exact single-axis rotations (matches
   `test_body_axisangle_orientation` at line 6257), `1e-6` for composed/numerical
   cases (matches frame tests at line 7517).

4. **`euler_seq_to_quat()` is available in tests** because it's a module-level
   function in `model_builder.rs`. AC3 uses it directly to compute the expected
   value (pattern from `test_euler_seq_zyx_body_orientation`).

5. **Run with:** `cargo test -p sim-mjcf`

#### Files

- `sim/L0/mjcf/src/model_builder.rs` — 16 acceptance tests (~250 lines, tests only)

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
