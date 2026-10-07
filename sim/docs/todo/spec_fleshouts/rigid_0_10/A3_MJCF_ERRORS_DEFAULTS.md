> Appendix to `RIGID_SPEC.md`, written during planning by a read-only researcher at `3520544e`. Paths under `$SCRATCH` are the planning session's scratch and are **not in the repo**; every claim here is re-established by the tests its commit adds. Where this appendix and `RIGID_SPEC.md` disagree, `RIGID_SPEC.md` wins.

# Rigid spec — sim-mjcf errors, default classes, and where validation runs

Area rows: mjcf-E1, mjcf-E2, mjcf-S10, ledger-L26, mjcf-S3, mjcf-S11, the "where validation runs" architecture question, and sim-mjcf's `config.rs` under core-D4.

Repo `main` @ `3520544e`, `git status --short` empty before and after this work. Scratch: `SCRATCH/rigid_spec/mjcf_errors_defaults/`. It holds `cases/` (62 probe docs), `ours_cases*.txt` and `mj350_cases.txt` (results), `errsites.tsv` (every error construction), `msgtests.tsv` (tests that assert on message text), `scripts/`, and `probe/` (a scratch Cargo project with path deps on sim-mjcf and sim-core). `locproto/` prototypes E2.

**How the evidence was gathered.** "Measured" means I ran it. Each case doc went through our loader (`load_model` or `load_model_from_file` on main, one process per file, 20 s timeout, 2.5 GB RSS watchdog polled every 100 ms) and through MuJoCo 3.5.0 (`mujoco.MjModel.from_xml_string`, `SCRATCH/rigid_oracle_350/venv`). MuJoCo citations are to `SCRATCH/mj350src/mujoco` @ tag 3.5.0 (881544c). The ledger-L26 hang input was **not** run through our loader. Every corpus scanner here was made to report a positive before I trusted its zero, and each scan's limits are stated where it is used.

**What these methods cannot see:**
- 157 `format!` templates and runtime-generated MJCF. The corpus is the 1,584 static docs.
- Behaviour on a branch: I changed no code. Every corpus count of "would flip" is a scan or a reading, not a run of the new loader.
- The submodule `.xml` files (not in the corpus).

---

## 1. mjcf-E1 — the error type (decision 2). This is the PR's FIRST mjcf commit.

### Now
- **Error types.** `MjcfError` (`error.rs:6-197`) is a 27-variant enum: 23 variants plus 4 behind `cfg(feature="mjb")`. It is not `#[non_exhaustive]`. It mixes tuple variants and struct variants. `InvalidAttribute.attribute` is `&'static str` (`error.rs:34`), and so is `MissingAttribute.attribute` (`:25`).
  - The builder has its own `pub struct ModelConversionError { pub message: String }` (`builder/mod.rs:70-82`). `model_from_mjcf` returns it (`:224-227`).
- **Where typed errors get flattened.**
  - `validate()` errors are flattened into it with `format!("Model validation failed: {e}")` at `:234-236`.
  - Tendon validation (`:299-301`) and user-data errors (`:330-332`) are flattened the same way.
  - Composite errors are flattened with `e.to_string()` at `:252-254`.
  - `load_model` and `load_model_from_file` then flatten everything again into `MjcfError::Unsupported(e.message)` (`:374`, `:411`).
  - A file read failure becomes `Unsupported(format!("failed to read file …"))` (`:396-402`). The `Io(#[from] std::io::Error)` variant (`error.rs:151`) has no path. Only `mjb.rs:150,244` reach it, via `File::create/open(path)?`.
- **Measured on main** (`ours_cases.txt`):
  - `e01` duplicate body: `Unsupported("Model validation failed: duplicate body name: a")`.
  - `e06` `<composite type="grid">`: `Unsupported("unsupported MJCF feature: The \"grid\" composite type is deprecated…")`. Its Display carries the prefix **twice**.
  - `e05` bad fixed-tendon joint: `Unsupported("Tendon validation failed: invalid option 'tendon': Tendon 't' references unknown joint 'nope'")`. Tendon errors are misfiled as `InvalidOption`.
  - `e02` `size="0.1 x"` on line 4: `XmlParse("invalid float: x")`, with no location.
- **Inventory.** `scripts/errsites.py` produces `errsites.tsv`. It covers non-test code only; a file's test region starts at `#[cfg(test)] mod … {`.
  - It found **334 error constructions**: 219 `MjcfError::…` and 115 `ModelConversionError { … }`. The 336 rows include 2 that are trait impl lines.
  - By current variant: `XmlParse` 100, `invalid_option` 29, `Unsupported` 19, `missing_attribute`/`MissingAttribute` 21, `invalid_attribute` 11, `IncludeError` 9, `DuplicateInclude` 3, `UnknownGeomType` 2, `UnknownJointType` 1, `InvalidFluidShape` 2, `InvalidFluidCoef` 2, `InvalidElement` 2, `invalid_mass` 2, `invalid_inertia` 2, `invalid_geom_size` 2, `undefined_site` 2, `undefined_joint` 1, `missing_element` 1, `Duplicate{Body,Joint,Actuator}` 1 each, MJB 6.
  - **Dead variants** (zero non-test constructions, directly or through a helper): `KinematicLoop`, `UnknownActuatorType`, `UndefinedBody`, `UndefinedGeom`, `UndefinedMesh`.
  - **`XmlParse` is overloaded:**
    - 33 quick-xml syntax errors (`e.to_string()`) and 31 "unexpected EOF in X";
    - `invalid float`/`invalid integer` (`parser/attrs.rs:22,100`) and wrong vector length (`attrs.rs:72,86`);
    - 13 hfield and mesh rules (`parser/asset.rs:121-222`: missing attributes, counts, value rules, one keyword), `maxhullvert` (`attrs.rs:61`), and 4 in `parser/defaults.rs:349-436` (3 tendon-default `springlength` rules, 1 mesh-inertia keyword);
    - 11 include writer, UTF-8 and parse errors (`include.rs:85-270`);
    - plugin config (`parser/extension.rs:107,145`) and joint-in-spatial-tendon (`parser/tendon.rs:80`).

### Downstream (git grep, workspace, excluding `*.md`)
- **Nothing outside sim-mjcf constructs `MjcfError`.**
  - `sim/L0/therm-env/src/error.rs:53` wraps it with `Mjcf(#[from] sim_mjcf::MjcfError)` and `#[error(transparent)]`, and `therm-env/src/builder.rs:339` uses `?`. Both compile unchanged.
  - `sim/L0/urdf/src/lib.rs:155` maps it with `format!("MJCF conversion error: {e}")`. Compiles unchanged.
  - `sim/L0/tests/integration/fluid_forces.rs:1197,1217` match `MjcfError::InvalidFluidShape(_)` and `InvalidFluidCoef(_)`. These must be rewritten.
- `ModelConversionError` has 0 code uses outside sim-mjcf. `design/cf-design/src/mechanism/builder.rs:246` names it in a doc comment, which needs updating.
- `sim_mjcf::model_from_mjcf` has 0 callers outside sim-mjcf. Same-named local helpers in `newton_solver.rs`, `noslip.rs` and others are different functions.
- Every other downstream use is `load_model(..).expect(..)` or `{e}`, which is type-transparent. The callers are cf-design-tests, sim-bevy, sim-coupling tests, and therm-env.
- `sim` re-exports the crate as `sim::mjcf` (`sim/sim/src/lib.rs:100`).

### Target

```rust
// sim/L0/mjcf/src/error.rs
use std::{fmt, path::PathBuf};

/// Errors from reading MJCF and building a `sim_core::Model` from it.
/// Each message ends with where the problem was found, when known ([`Location`]).
/// Every variant is `#[non_exhaustive]`: match with `{ .. }`.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum MjcfError {
    // ── reading input ──
    #[error("cannot read '{}': {source}", .path.display())]
    #[non_exhaustive] Io { path: PathBuf, source: std::io::Error },
    #[error("XML syntax error: {message}{at}")]
    #[non_exhaustive] Xml { message: String, at: Location },
    #[error("include error: {message}{at}")]
    #[non_exhaustive] Include { message: String, at: Location },
    #[error("elements nested deeper than {limit}{at}")]
    #[non_exhaustive] TooDeep { limit: usize, at: Location },                 // mjcf-H2
    // ── schema (mjcf-S1) ──
    #[error("unrecognized element <{element}> inside <{parent}>{at}")]
    #[non_exhaustive] UnknownElement { element: String, parent: String, at: Location },
    #[error("unrecognized attribute '{attribute}' on <{element}>{at}")]
    #[non_exhaustive] UnknownAttribute { element: String, attribute: String, at: Location },
    #[error("<{element}> may appear at most once inside <{parent}>{at}")]
    #[non_exhaustive] RepeatedElement { element: String, parent: String, at: Location },
    #[error("missing required element <{element}> inside <{parent}>{at}")]
    #[non_exhaustive] MissingElement { element: String, parent: String, at: Location },
    #[error("required attribute missing: '{attribute}' on <{element}>{at}")]
    #[non_exhaustive] MissingAttribute { element: String, attribute: String, at: Location },
    #[error("<{element}> may set only one of: {}{at}", .attributes.join(", "))]
    #[non_exhaustive] ConflictingAttributes { element: String, attributes: Vec<String>, at: Location },
    // ── attribute values (mjcf-S2, decision 3) ──
    #[error("bad format in attribute '{attribute}' on <{element}>: \"{value}\"{at}")]
    #[non_exhaustive] BadNumber { element: String, attribute: String, value: String, at: Location },
    #[error("attribute '{attribute}' on <{element}> has {found} value(s), expected {expected}{at}")]
    #[non_exhaustive] WrongValueCount { element: String, attribute: String, expected: ValueCount, found: usize, at: Location },
    #[error("invalid keyword \"{value}\" in attribute '{attribute}' on <{element}>{at}")]
    #[non_exhaustive] InvalidKeyword { element: String, attribute: String, value: String, at: Location },
    #[error("attribute '{attribute}' on <{element}> is not finite ({value}){at}")]
    #[non_exhaustive] NonFinite { element: String, attribute: String, value: f64, at: Location },
    #[error("invalid {attribute} on <{element}>: {reason}{at}")]
    #[non_exhaustive] InvalidValue { element: String, attribute: String, reason: String, at: Location },
    // ── names, references, default classes ──
    #[error("repeated name '{name}' in {kind}{at}")]
    #[non_exhaustive] DuplicateName { kind: ElementKind, name: String, at: Location },
    #[error("unknown {kind} '{name}'{at}")]
    #[non_exhaustive] UndefinedReference { kind: ElementKind, name: String, at: Location },
    #[error("unknown default class name '{name}'{at}")]
    #[non_exhaustive] UnknownClass { name: String, at: Location },
    #[error("empty class name: a nested <default> needs class=\"…\"{at}")]
    #[non_exhaustive] EmptyClassName { at: Location },
    #[error("top-level default class 'main' cannot be renamed (class=\"{name}\"){at}")]
    #[non_exhaustive] RenamedRootClass { name: String, at: Location },
    // ── model-level compiler rules, assets ──
    #[error("{reason}{at}")]
    #[non_exhaustive] InvalidModel { reason: String, at: Location },
    #[error("{kind} '{name}': {reason}{at}")]
    #[non_exhaustive] Asset { kind: ElementKind, name: String, reason: String, at: Location },
    // ── stated limitation (decision 1): valid MuJoCo 3.5.0 this crate does not implement ──
    #[error("not supported by sim-mjcf (valid in MuJoCo 3.5.0): {feature}{at}")]
    #[non_exhaustive] Unsupported { feature: String, at: Location },
    // ── .mjb ──
    #[cfg(feature = "mjb")] #[error("not an MJB file: magic bytes {found:?}, expected \"MJB1\"")]
    #[non_exhaustive] MjbMagic { found: [u8; 4] },
    #[cfg(feature = "mjb")] #[error("unsupported MJB version {found} (supported: 1)")]
    #[non_exhaustive] MjbVersion { found: u32 },
    #[cfg(feature = "mjb")] #[error("MJB encode error: {message}")]
    #[non_exhaustive] MjbEncode { message: String },
    #[cfg(feature = "mjb")] #[error("MJB decode error: {message}")]
    #[non_exhaustive] MjbDecode { message: String },
}

/// Which kind of named element an error is about. Displays as the MJCF tag ("body", …).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum ElementKind { Body, Frame, Joint, Geom, Site, Camera, Light, Mesh, Hfield, Skin, Texture,
                       Material, Flex, Pair, Exclude, Equality, Tendon, Actuator, Sensor, Keyframe,
                       Plugin, DefaultClass }

/// How many values an attribute takes. Displays "exactly 3" / "at most 6" / "between 1 and 3".
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ValueCount { Exactly(usize), AtMost(usize), Between(usize, usize) }

/// Where an error was found. Opaque and Display-only (mjcf-E2 is message-only);
/// accessors can be added later without a breaking change.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Location(Option<Box<LocationInner>>);          // LocationInner is private
impl fmt::Display for Location { /* "" | " (line 4: worldbody > body 'a' > geom)" | " (geom 'g' in body 'arm')" */ }

pub type Result<T> = std::result::Result<T, MjcfError>;
```

**Public signatures that change:**
```rust
pub fn model_from_mjcf(mjcf: &MjcfModel, base_path: Option<&Path>) -> Result<Model>;   // was Result<Model, ModelConversionError>
// deleted: pub struct ModelConversionError (builder/mod.rs:70-82; re-export lib.rs:209)
// deleted: helper ctors MjcfError::{missing_element, missing_attribute, invalid_attribute, undefined_*, invalid_*} (error.rs:199-300) → pub(crate)
```

**Decisions inside E1, with reasons:**
- **`String` for every element and attribute field, `&'static str` nowhere.**
  - An unknown attribute is arbitrary text, so `&'static str` cannot hold it. That is why the old `InvalidAttribute.attribute: &'static str` (`error.rs:34`) cannot name one.
  - Some "attributes" are compound, for example `"joint/tendon/site/body"` at `validation.rs:353`. Those become `ConflictingAttributes.attributes: Vec<String>`.
  - One field type across variants means one pattern-matching style. The cost is one allocation per error, on the error path only.
- **Variants are `#[non_exhaustive]`.**
  - Thermo-B left `ThermostatError` variants open because third parties construct them. Nothing outside sim-mjcf constructs `MjcfError` (grep above), so that reason does not apply here.
  - The gain is that fields can be added later without a breaking change: a typed source span, or a `source` on `Asset`.
  - The cost: outside code must match `{ .. }` (tuple patterns are not allowed on non-exhaustive variants, so every variant is struct-like), and cannot construct variants. Only `fluid_forces.rs:1197,1217` match today.
- **`Unsupported` has one meaning: valid MuJoCo 3.5.0 that this crate does not implement** (decision 1, "stated limitation"). Of today's 19 `Unsupported` sites, 13 are MuJoCo's own rules and move to typed variants:
  - the composite rules (`builder/composite.rs:45-164`). The deprecation texts are MuJoCo's: `e06` gives the same message as MuJoCo 3.5.0 ("The "grid" composite type is deprecated…");
  - `coordinate='global'` (`parser/options.rs:304`). MuJoCo also refuses it ("global coordinates no longer supported", seen in `corpus350.jsonl`);
  - `nuser` (`builder/mod.rs:851,860`). MuJoCo refuses too: "nuser_geom must be >= -1" and "user has more values than nuser_geom" appear in `corpus350.jsonl`.

  After E1, `Unsupported` is built at: `builder/joint.rs:54` (joint type not supported in Model), `builder/mesh.rs:223` (`hfields` feature off), and `parser/body.rs:566,594` (joint, freejoint or inertial in a `<frame>`; decision 5 deferred). The parser and validation areas add their limitations (12 dropped sensor types, MuJoCo-valid attributes never read, ball-then-slide).
- **`Io` carries the path.** It drops `#[from]`; there are 2 `?` sites in `mjb.rs:150,244`. It is produced by: `load_model_from_file`'s read (`builder/mod.rs:396`), include canonicalize/read (`include.rs:53,67,92,105`), and `mjb.rs:150,244`. Asset "file not found" checks (`builder/asset.rs:57,92`) are existence tests with no `io::Error`. They become `Asset { reason: "file not found…" }`, which keeps the substring asserted at `asset.rs:139,186` and `builder/mod.rs:1241`.
- **Mapping of the 334 sites (by group; `errsites.tsv` is the row-level list):**

| today | → new variant |
|---|---|
| `XmlParse(quick-xml e)` ×33, `"unexpected EOF in X"` ×31 | `Xml` |
| include write / UTF-8 / parse-include (`include.rs:85-270`) ×11; `IncludeError` ×9; `DuplicateInclude` ×3 | `Include`; resolve/read failures → `Io`; `<include>` without `file` (`include.rs:161`) → `MissingAttribute` |
| `invalid float` / `invalid integer` (`attrs.rs:22,100`); gravcomp (`parser/body.rs:232`) | `BadNumber` |
| `expected N values` (`attrs.rs:72,86`); hfield size count (`asset.rs:172`); `InvalidFluidCoef` (`parser/body.rs:417`, `parser/defaults.rs:207`) | `WrongValueCount` |
| `UnknownJointType`, `UnknownGeomType`, `InvalidFluidShape`; `invalid_attribute` for angle / eulerseq / inertiafromgeom / coordinate / composite type / curve; mesh inertia mode; interp, gaintype, biastype, dyntype (`builder/actuator.rs:143,682,697,714`, `builder/sensor.rs:87`) | `InvalidKeyword` |
| `invalid_option` for option checks (`validation.rs:44-134`); hfield/mesh/springlength/maxhullvert rules; keyframe length (`builder/mod.rs:108`); muscle and cranklength checks (`builder/actuator.rs:151-260,399`); sensor interval/delay (`builder/sensor.rs:95,104`); `nuser` | `InvalidValue` (NaN/inf ones → `NonFinite`) |
| `invalid_geom_size` (`validation.rs:387,395`), keyframe NaN (`builder/mod.rs:93,119`), `invalid_mass` NaN | `NonFinite` |
| `invalid_mass` ≤ 0, `invalid_inertia` (`validation.rs:224-239`) | `InvalidValue { element: "inertial", attribute: "mass" / "diaginertia" }` |
| `Duplicate{Body,Joint,Actuator}`; duplicate mesh/hfield (`builder/mesh.rs:47,95`) | `DuplicateName` |
| `undefined_joint` / `undefined_site` (`validation.rs:316-336`); ~45 "references unknown X" messages in `builder/{actuator,contact,equality,flex,geom,sensor,tendon}.rs`; tendon refs inside `invalid_option` (`validation.rs:457-702`) | `UndefinedReference` |
| childclass undefined (`builder/frame.rs:44,68`) | `UnknownClass` |
| mutual exclusivity (`validation.rs:353`; `builder/sensor.rs:56,63,401`; cranksite without slidersite `builder/actuator.rs:373`; composite vertex+count) | `ConflictingAttributes` |
| mocap placement (`builder/body.rs:108,116`), negative mesh volume (`builder/mesh.rs:65`), no transmission target (`builder/actuator.rs:408`), wrap geom type (`builder/tendon.rs:148`), tendon path structure (`validation.rs:539-645`) | `InvalidModel` |
| asset/mesh/hfield load and content (`builder/asset.rs:57,84,92`; `builder/mesh.rs:140-438`) | `Asset` |
| `builder/contact.rs:116` "contact_pair_set and contact_pairs out of sync" | an internal invariant, not an input error. Restructure so it cannot fail (a `HashMap<key, index>` instead of `HashSet` + `position`), and the site disappears. |
| flatteners `builder/mod.rs:234,252,299,330,374,411` | deleted; typed errors pass through |
| MJB ×6 + `mjb.rs:150,244` | `Mjb*`, `Io` |

### Tests to add (each fails on main)
1. `load_model` with two `<body name="a">` gives `DuplicateName { kind: ElementKind::Body, name == "a", .. }`. Main gives `Unsupported("Model validation failed: …")` (measured, `e01`).
2. `load_model_from_file(<tmpdir>/missing.xml)` gives `Io { path, source }` with `source.kind() == NotFound` and `path` equal to the argument. On main, by reading `builder/mod.rs:396-402`, it gives `Unsupported` (not run).
3. `<composite type="grid">` gives `InvalidValue { element: "composite", attribute: "type", .. }`, and the Display contains no "unsupported". Main doubles the prefix (measured, `e06`).
4. A fixed tendon naming joint `nope` gives `UndefinedReference { kind: Joint, name == "nope", .. }`. Main gives `Unsupported("Tendon validation failed: invalid option 'tendon': …")` (measured, `e05`).

### Tests that flip (all CI-run unless noted)
- **`lib.rs:293-310`** `test_validate_wired_into_model_from_mjcf` matches `Unsupported(msg) if msg.contains("validation failed")`. Rewrite it to `DuplicateName`.
- **Variant-name rewrites** (compile-level; the behaviour is unchanged):
  - in-crate: `validation.rs:747-1071` (27 match lines), `lib.rs:281`, `parser/tests.rs:194,1360`, `mjb.rs:436,444,571,584,595` (`--features mjb`, which CI does not run per the design's verification plan), `include.rs:427,449,471`, and `error.rs:310-364` (8 tests of the deleted helpers);
  - outside the crate: `sim/L0/tests/integration/fluid_forces.rs:1197,1217`. T39 at `:1205` pins a refusal of a 2-value `fluidcoef`. MuJoCo 3.5.0 **loads** it (measured, `cases/x_fluidcoef2.xml`), so whether T39 flips to Ok is the parser area's call (exact lengths, mjcf-S2). E1 only renames the variant.
- **Message-substring tests.** `msgtests.tsv` lists 81 `contains("…")` assertions that sit within 12 lines of an error token, in files that call a loader. The scanner was checked against `lib.rs:308`, which it finds. Eight of the 81 are not sim-mjcf messages: urdf-loading example ×5, fsu-model ×2, coupling ×1.
  - The new Display texts must keep these substrings: "unknown tendon", "unknown body", "unknown" + the name (`equality_constraints.rs:764,1106`), "not found", "relative", "no base path", "nonexistent", "ghost", "mocap", "finite", "setting delay > 0 without a history buffer", "negative interval in sensor", "invalid interp keyword", "must be slide or hinge", "must be different", "deprecated", "one-dimensional", "Cannot specify both", "vertex and count", "small", "for mesh geoms", "mesh volume is negative", "maxhullvert must be larger than 3", "duplicate config key", "coordinate='global'", "eulerseq", "nuser_geom", "hfields"/"feature", and the six tendon-path phrases in `spatial_tendons.rs:1019-1161`.
  - With those kept, only `lib.rs:308` flips on text.
  - The scanner cannot see `assert_eq!` on a whole message, or `contains` more than 12 lines away from the error token.

---

## 2. mjcf-E2 — locations, message-only

### Now
No error carries a line, element path, or attribute position. `e02` on main reads "XML parse error: invalid float: x" (measured). MuJoCo 3.5.0 appends `"\nElement 'geom', line 4"` to reader errors (`xml_util.cc:258-275`). Compiler errors get `"Element name 'x', id N, line L"` (`user_objects.cc:194-230`), because the reader stores `"line N"` in each object's `info` (`xml_native_reader.cc:1833,1907,1943,…`).

### Target: two cheap tiers, no new fields on the `Mjcf*` structs
1. **Errors raised while parsing** carry a line number plus an element path built from tag names and `name` attributes.
   - `parse_mjcf_str` (`parser/mod.rs:50-54`) already owns the input `&str` and the `Reader`. On `Err` it takes `reader.buffer_position()`, or `error_position()` for quick-xml syntax errors.
   - Attribute errors are raised in the `Start`/`Empty` arm before the next read, and propagate with `?` (by reading, `parser/body.rs`, `parser/actuator.rs`). So that offset is the end of the offending start tag. `rfind('<')` gives its opening line.
   - A second iterative scan of the prefix (an explicit stack, so deep nesting is safe for mjcf-H2) builds a path such as `worldbody > body 'a' > geom`. The path is capped to its last 8 segments.
   - The result is set on the error by one `pub(crate) fn with_location_if_unknown(self, ..)` in `error.rs`: an exhaustive match that fills `at` on variants that have it.
   - **Prototype measured** (`locproto/`, quick-xml 0.41.0, the repo's `Cargo.lock`):
     - `e02` gives "line 4 path mujoco/worldbody/body[a]/geom". MuJoCo says "line 4".
     - `e07` (mismatched close) gives line 5, the `</body>` line. MuJoCo says line 4, where the element opened.
     - `e08` (unclosed quote) gives line 3, the body tag. MuJoCo says line 4.
   - Zero edits to the ~260 parse call sites.
2. **Errors raised while resolving, validating or building** carry `kind 'name'` plus the enclosing body, or `kind #k` when the element is unnamed. They are built by `Location::element(ElementKind, Option<&str>, Option<&str>)` from data the pass already holds.

### Limits, stated rather than fixed
- **Line accuracy.** Lines are exact for errors raised on the element's start tag. Errors raised after more events have been read point at the last event read. I did not enumerate which parse sites do that; candidates are `End` arms and checks after a loop.
- **Include files.** In `load_model_from_file`, lines refer to the **include-expanded** text (`include.rs` splices text), not to the included file. Fixing that needs `expand_includes` to return a line map (expanded line → file, line).
- **Lines on build-stage errors** would need a `line: Option<u32>` field on about 20 pub structs (`MjcfBody`, `MjcfGeom`, …), as MuJoCo's `info` does. That is the "typed spans (large)" option, and it is deferred.

### Tests to add (fail on main)
- `e02` gives a Display containing "line 4" and "body 'a'". Main: "XML parse error: invalid float: x" (measured).
- An unnamed `<motor joint="nope">` gives a Display naming the actuator (`actuator #0`) and the missing joint. Main: "… in actuator ''" (measured, `e04`).

---

## 3. Default classes

### mjcf-S10 — the root class "main", repeated top-level defaults, unknown classes

#### Now (code)
- **Parsing.** Every top-level `<default>` becomes `MjcfDefault { class: attr or "", parent_class: None }` (`parser/defaults.rs:26-31`, `parser/mod.rs:94-97`). Nested ones get `parent_class: Some(parent's class)` (`:68-72`).
- **Resolution.** `resolve_all` keys a `HashMap` by class (`defaults.rs:752-771`), so a repeated top-level block replaces the earlier one (last wins). `resolve_single` walks parent links at the time of resolution (`:774-802`).
- **Lookup.** A missing class returns `None`, and the element then gets **no defaults at all, not even the root's** (`defaults.rs:73-76`, `apply_to_*`).
- Only `childclass` is checked (`builder/frame.rs:37-82`).
- A self-closing nested `<default class="a"/>` is ignored: the `Empty` arm at `parser/defaults.rs:76-106` has no `default` case.
- Within one `<default>`, a second actuator shortcut replaces the first (`:46-49`, `:83-86`).

#### Measured, ours vs MuJoCo 3.5.0 (`ours_cases.txt` vs `mj350_cases.txt`)

| case | ours (main) | MuJoCo 3.5.0 |
|---|---|---|
| d01 top-level `<default class="main">` damping 2 | damping 0 | 2 |
| d02 element `class="main"` | 0 | 2 |
| d03 body `childclass="main"` | Err `Unsupported("childclass 'main' … undefined")` | loads, 2 |
| d04 two top-level blocks (damping 2 / armature .5) | 0 / 0.5 | 2 / 0.5 (merge) |
| d05 class "a" made in block 1, root armature 2 in block 2 | armature 2 | **0** (snapshot at creation) |
| d05b nested class before the parent's own `<joint>` in one block | 1 / 2 | 1 / 2 |
| d06 `class="nope"` | loads, **root damping lost** (0) | "unknown default class name 'nope'" |
| d07 top-level `class="foo"` | loads, root lost | "top-level default class 'main' cannot be renamed" |
| d08 sibling classes both "a" | loads | "repeated default class name" |
| d09 nested `class="main"` | loads | "repeated default class name" |
| d10 two `<joint>` in one default | 2nd replaces 1st | "Schema violation: unique element 'joint' found 2 times" |
| d11 `<motor gear=5/><position kp=10/>` in one default | gear 1 (motor default lost) | gear 5 on both actuators |
| d12 `class=""` | treated as root | "unknown default class name ''" |
| d13 `childclass="nope"` | Err | Err (texts differ) |
| d15 self-closing nested `<default class="a"/>` | joint gets damping 0 | 2 |
| x_crosstalk `<default><position kp=10/>` + `<velocity>` / `<general>` | kv 1 / gain 1 | **kv 10** / gain 10 + bias (one shared actuator default) |

#### MuJoCo 3.5.0 source
- **Reading order.** All `<default>` sections are read before every other section (`xml_native_reader.cc:1017-1022`; worldbody at `:1077`).
- **`Default()`** (`:2884-2973`):
  - nested with empty class gives "empty class name" (`:2891-2895`);
  - repeated name gives "repeated default class name" (`:2896-2900`);
  - top level must be `""` or `"main"`, else "top-level default class 'main' cannot be renamed" (`:2901-2905`);
  - the block's own elements are read first (`:2908`), then nested defaults (`:2961`);
  - every actuator shortcut writes the one `def->actuator`.
- **Snapshot.** A nested class copies its parent **when it is created** (`user_model.cc:1499-1520`, `CopyWithoutChildren` at `:1516`).
- **Root name and lookup.** The root is named "main" (`user_model.cc:176`), and `FindDefault` matches by name (`:1487-1495`), so `class="main"` and `childclass="main"` resolve to the root.
- **Unknown names.** `GetClass` errors on an unknown name, including `""` (`xml_native_reader.cc:4558-4571`). An unknown childclass errors at `:3631-3634`, `:3679-3682` and `:3732-3735`.
- **Schema.** Each element under `<default>` is `"?"`, at most once (`xml_native_reader.cc:151-206`). There is no `sensor` element there.

#### Target
Parity for every row above, with these choices:
- **The resolver is rewritten as ordered, snapshot resolution.** It no longer walks parent chains, so ledger-L26's loop cannot exist. The signature becomes fallible:
  ```rust
  pub(crate) struct DefaultResolver { classes: HashMap<String, MjcfDefault> }   // key "" = root ("main")
  impl DefaultResolver {
      /// Resolve `defaults` in document (pre-)order, as MuJoCo does: a nested class copies
      /// its parent's state at the moment it is created (user_model.cc:1516); a top-level
      /// block (parent_class None) merges into the root (xml_native_reader.cc:2901-2905).
      pub(crate) fn new(defaults: &[MjcfDefault]) -> Result<Self>;
      /// None and "main" → root; "" or a missing name → `UnknownClass`.
      pub(crate) fn class(&self, name: Option<&str>) -> Result<&MjcfDefault>;
  }
  ```
  - For each entry in slice order: `parent_class == None` with a class outside {"", "main"} gives `RenamedRootClass`. Otherwise the entry is merged into the root.
  - A nested entry with an empty class gives `EmptyClassName` (this is ledger-L26).
  - A class equal to "main" or already defined gives `DuplicateName { kind: DefaultClass }`.
  - A parent that does not exist yet gives `UnknownClass`. Only caller-built lists can trigger it.
  - Otherwise `classes[class] = merge(classes[parent].clone(), entry)`. The parser emits blocks in pre-order with each block's own elements first (`parser/defaults.rs:115-117`), which yields MuJoCo's semantics (d05, d05b).
- **The root keeps the stored name `""`.** `"main"` is accepted as an alias on lookup, and the parser normalizes a top-level `class="main"` to `""`. This keeps `MjcfDefault::default()` and `parser/tests.rs:892-902` (`parent_class == Some("")`) unchanged.
- **Parser fixes:**
  - handle a self-closing nested `<default class=…/>` (d15) in the `Empty` arm;
  - merge repeated actuator-shortcut elements in one block by attribute overlay instead of replacing (d11).
  - d10 (the same element twice) is a schema error and belongs to the mjcf-S1 schema pass. If S1 does not implement multiplicity, a seen-set in `parse_default` produces `RepeatedElement`.
- **Unknown classes are checked in the resolve pass (§4).** That covers `class=` on every element and `childclass=` on bodies and frames, and replaces `validate_childclass_references` (`builder/frame.rs:32-82`).
- **`DefaultResolver` is no longer exported** (`lib.rs:193`). It has 0 uses outside sim-mjcf (git grep). Exporting it again later is an additive change; keeping it exported now freezes `apply_to_*` and the getters. Open question O-3.

#### Tests to add (each fails on main; outcomes measured above)
d01, d02, d03, d04, d05, d06 (gives `UnknownClass`), d07 (`RenamedRootClass`), d08 and d09 (`DuplicateName { DefaultClass }`), d12 (`UnknownClass { name: "" }`), d15, d11.

#### Tests that flip
- `sim/L0/tests/integration/default_classes.rs:276-298` `test_nonexistent_class_no_panic` now gets `Err(UnknownClass)` (CI).
- `defaults.rs` unit tests that build a top-level *named* class (`parent_class: None` with class "test", "visual", "motor", "cable", "noisy", "existing"; at `:1210,1249,1275,1302,1326,1348,1368,1411`) must nest it under the root (`parent_class: Some(String::new())`). Otherwise `new` returns `RenamedRootClass`.
- `test_apply_to_actuator` (`:1273`) and the sensor test (`:1407`) also change with mjcf-S11's `Option` fields.
- `builder/frame.rs` AC33/AC35 (`:1147-1215`) assert `is_err()` + `contains(name)`. They keep passing if `UnknownClass` Display carries the name.

### ledger-L26 — a classless nested default hangs

- **Now.** `<default><joint/><default><joint/></default></default>` gives the nested entry class `""` and parent `Some("")` (`parser/defaults.rs:26,70`). `resolve_single`'s `while let Some(default) = raw_map.get(current)` (`defaults.rs:779-785`) then follows `""` → `""` forever, pushing onto `chain`. The planning agent measured that it did not return in 120 s. **I did not run it.**
- **Target.** `EmptyClassName`, which is MuJoCo's "empty class name" (`xml_native_reader.cc:2891-2895`).
- **Change.** The resolver rewrite above; there is no chain walk left.
- **Test.** Assert `EmptyClassName` for that input, and `DuplicateName { DefaultClass }` for `<default><default class="a"><default class="a"/></default></default>`, the second hang shape by reading ledger-L26.
  - ⚠ If this test regresses to a chain walk, it grows memory without bound. Fail-fast with a timeout does not bound RAM. The safeguard is that the new code has no loop that can revisit an entry.

### mjcf-S3 — geom `size` in defaults, element-wise overlay, MuJoCo's default size 0

#### Now
- `MjcfGeomDefaults` has no `size` (`types.rs:676-735`), and `parse_geom_defaults` does not read it.
- `MjcfGeom.size: Vec<f64>` defaults to `vec![0.1]` (`types.rs:1268,1332`).
- Missing components fall back to 0.1 or to the first value (`builder/geom.rs:373-509,857-884`; `types.rs:1408-1419`).

#### Measured

| case | ours | MuJoCo 3.5.0 |
|---|---|---|
| s3_01 default `size="0.05"` + bare sphere | 0.1 | 0.05 |
| s3_02 default capsule `0.05 0.3` + element `size="0.07"` | (0.07, 0.1) | (0.07, 0.3) |
| ov_class_size_chain, same through a class | (0.1, 0.1) | (0.07, 0.3) |
| s3_08 class vs root sizes | 0.1 / 0.1 | 0.05 / 0.2 |
| s3_03, s3_04, s3_06, s3_07, s3_09, s3_10, s3_11 (bare or zero sizes) | load with 0.1 or 0 | "size N must be positive in geom" |
| s3_05 plane with no size | loads (0.1, 0.1, 0.1) | "plane size(3) must be positive" |
| s3_12 ellipsoid with 2 values | (0.1, 0.2, 0.2) | "size 2 must be positive in geom" |

#### MuJoCo 3.5.0 source
- **Default size 0.** The geom default is memset to 0 (`user_init.c:109-110`).
- **Overlay.** `size` is read with `exact=false` onto the class copy (`xml_native_reader.cc:1854`; `ReadAttr` at `xml_util.cc:798-817`), so a short value overlays element-wise.
- **Size check.** `checksize` (`user_objects.cc:152-167`, called at `:3758` after mesh fitting and fromto) refuses `size[i] <= 0` for `i < mjGEOMINFO[type]` (`user_objects.h:77`). For a plane it checks only `size[2]`.
- **Other `exact=false` attributes overlay the same way** (`xml_native_reader.cc`):
  - joint `solreflimit`, `solimplimit`, `solreffriction`, `solimpfriction` (1807-1810);
  - geom `friction`, `solref`, `solimp`, `fluidcoef` (1860-1880);
  - site `size` (1928);
  - pair `solref`, `solreffriction`, `solimp`, `friction` (2083-2088);
  - equality `polycoef`, `solref`, `solimp` (2199-2232);
  - tendon `solref*`, `solimp*`, `springlength` (2258-2270);
  - actuator `gear` (2307) and `dynprm`/`gainprm`/`biasprm` (2387-2389).

  Measured: `gear="5"` under class gear `1 2 3 4 5 6` gives MuJoCo [5,2,3,4,5,6] (`x_gear_overlay`); ours [5,0,0,0,0,0] by reading `parser/actuator.rs:70-76`. Geom `friction="0.7"` under default `0.5 0.01 0.001` gives MuJoCo (0.7, 0.01, 0.001); ours refuses ("expected 3 values in vector, got 1") (`ov_friction`).

#### Target (parity)
- The default geom size is `[0;3]`.
- A `size` in a default class overlays its parent's resolved size element-wise.
- An element `size` overlays its class's size element-wise.
- A bare `<geom/>`, a sizeless primitive, or a zero size is refused by MuJoCo's `checksize` rule, as `InvalidValue { element: "geom", attribute: "size", reason: "size 0 must be positive" }`.

#### Change
```rust
pub struct MjcfGeom         { pub size: Option<Vec<f64>>, /* was Vec<f64>, default vec![0.1] */ .. }
pub struct MjcfGeomDefaults { pub size: Option<Vec<f64>>, /* new */ .. }
pub(crate) fn overlay<const N: usize>(base: [f64; N], partial: &[f64]) -> [f64; N];   // shared by every exact=false attribute
```
- Delete the `unwrap_or(0.1)` fallbacks. `geom_size_to_vec3` and `MjcfGeom::computed_mass` take the resolved `[f64; 3]`.
- The helper serves all the `exact=false` attributes. **Whether partial arrays are accepted at all is the parser area's call** (mjcf-S2, exact lengths). The two areas must agree.
- **Placement of `checksize`.** It belongs with S3: changing the default to 0 without it would let a bare geom load as a zero sphere. **It must not land before the composite `curve` keyword fix (parser area).** Our cable generator misreads `curve="s"`; see §4. With checksize in place, `sim/L0/tests/integration/composite.rs:24-60` `t1_cable_basic_generation` (CI; `curve="s 0 0"`, corpus doc `263f733b2df7d3fe`) would be refused. Measured on main: its capsules have `geom_size[1] = 0.0`, and RULECHK `size_nonpos=4`. MuJoCo loads it with half-length 0.125.

#### Tests
- **Add:** s3_01, s3_02, ov_class_size_chain, s3_08 (values), s3_03/s3_05/s3_12 (refusals), all measured above.
- **Flip:**
  - compile-level: `types.rs:4441-4465`, `validation.rs:1025,1039`, `defaults.rs:1261`, `builder/geom.rs:898`, and `builder/composite.rs:474-516` (code);
  - behavioural: none found in the static corpus. `scripts/sizescan.py` found 0 docs with a sizeless primitive geom outside `<default>`, `<composite>` and tendon paths. Its positive control was 7/7 positives.
  - It cannot see `format!` templates or runtime MJCF. cf-design emits only mesh geoms (`design/cf-design/src/mechanism/mjcf.rs:370,383`). sim-urdf always writes `size` (`sim/L0/urdf/src/converter.rs:511`); a URDF radius of 0 would now be refused (not measured).

### mjcf-S11 — an explicit value equal to the built-in default is overwritten (decision 4)

#### Now
`apply_to_actuator` replaces a field with the class value whenever the field equals the built-in default (`defaults.rs:345-527`, with a `#todo` at `:345-348`). Same in `apply_to_sensor` (`:672-684`).

| struct | field | today's type | test in `apply_to_*` |
|---|---|---|---|
| `MjcfActuator` (`types.rs:2663`) | `gear` | `[f64; 6]` | == [1,0,0,0,0,0] |
| | `kp` | `f64` | == 1.0 |
| | `area` | `f64` | == 1.0 |
| | `bias` | `[f64; 3]` | == [0,0,0] |
| | `muscle_timeconst` | `(f64, f64)` | == (0.01, 0.04) |
| | `range` | `(f64, f64)` | == (0.75, 1.05) |
| | `force` | `f64` | == −1 |
| | `scale` | `f64` | == 200 |
| | `lmin` | `f64` | == 0.5 |
| | `lmax` | `f64` | == 1.6 |
| | `vmax` | `f64` | == 1.5 |
| | `fpmax` | `f64` | == 1.3 |
| | `fvmax` | `f64` | == 1.2 |
| | `gain` | `f64` | == 1.0 |
| `MjcfSensor` | `noise` | `f64` | == 0 |
| | `cutoff` | `f64` | == 0 |

- **No other struct uses the pattern.** `apply_to_joint`, `_geom`, `_site`, `_tendon`, `_pair`, `_mesh` and equality all use `Option` (`defaults.rs:160-337,540-750`). The one exception is `user` (`.is_empty()`, in 6 `apply_to_*`). MuJoCo's handling of an explicit `user=""` was **not checked**.
- **Measured, ours vs MuJoCo:**
  - `s11_gear` (class 50, element 1): 50 vs 1.
  - `s11_kp`: 50 vs 1.
  - `s11_area`: 5 vs 1.
  - `s11_bias` (class 1 2 3, element 0 0 0): [1,2,3] vs [0,0,0].
  - `s11_adhesion_gain`: 5 vs 1.
  - `s11_muscle2` (every muscle field set explicitly to its built-in default): ours takes the class's 9 gain params and timeconst; MuJoCo keeps the explicit values, except `force`.
  - `s11_sensor_noise` (explicit 0 under class noise .5 / cutoff 2): ours 0.5 / 2. MuJoCo refuses the doc: `<default><sensor>` is "Schema violation: unrecognized element".

#### Target
`Option<T>` on all 16 fields. Explicit `Some` wins, `None` takes the class, and the built-in default applies last, in the resolve pass. Sensor defaults stay as an allowlisted extension (decision 4 recommendation; the S1 allowlist owns the schema exception).

#### Change
```rust
pub struct MjcfActuator {
    pub gear: Option<Vec<f64>>,             // up to 6, overlaid element-wise (S3 helper); MuJoCo xml_native_reader.cc:2307
    pub kp: Option<f64>, pub area: Option<f64>, pub bias: Option<[f64; 3]>,
    pub muscle_timeconst: Option<(f64, f64)>, pub range: Option<(f64, f64)>,
    pub force: Option<f64>, pub scale: Option<f64>, pub lmin: Option<f64>, pub lmax: Option<f64>,
    pub vmax: Option<f64>, pub fpmax: Option<f64>, pub fvmax: Option<f64>, pub gain: Option<f64>, ..
}
pub struct MjcfSensor { pub noise: Option<f64>, pub cutoff: Option<f64>, .. }
```
- `MjcfActuatorDefaults.gear` also becomes `Option<Vec<f64>>`.
- Call sites: grep counts of lines naming any of these field names are `builder/actuator.rs` 16, `parser/actuator.rs` 14, `parser/tests.rs` 31 and `defaults.rs` 58. These are upper bounds: `.range` and `.force` also match same-named fields of other structs. There are 0 outside sim-mjcf.

#### MuJoCo's muscle has its own sentinel
`mjs_setToMuscle` (`user_api.cc:1009-1050`) treats an element value **< 0 as "not given"** (`:1033-1039`).
- Measured, `s11_muscle3_force77`: explicit `force="-1"` under a class with force 77 gives MuJoCo **77**.
- Measured, `m01_muscle_neg`: `lmin="-0.5"`, `scale="-3"` and `timeconst="-1 0.05"` are silently replaced by 0.5, 200 and 0.01. Ours stores −0.5, −3 and −1.

Plain `Option` matches MuJoCo except for negative muscle values. Open question O-4.

#### Tests
- **Add:** s11_gear, s11_kp, s11_area, s11_adhesion_gain, s11_muscle2, s11_sensor_noise. Each fails on main (measured above).
- **Flip:** compile-level in `defaults.rs:1273-1298,1407-1446` and `parser/tests.rs`. No behavioural flips found. Not searched beyond these files and `msgtests.tsv`.

---

## 4. Where validation runs — recommendation (b), a resolved-tree pass

### Now: the pipeline (`builder/mod.rs:224-356`)
1. clone;
2. `validate()` on the **as-parsed** tree (`:234`);
3. `DefaultResolver` + childclass check (`:240-241`);
4. `expand_frames` (`:246`). Frames compose transforms into elements **before** defaults exist, and push childclass into `class` fields (`builder/frame.rs:90-260`);
5. `expand_composites` (`:251-255`);
6. `apply_discardvisual` / `apply_fusestatic` (`:257-262`), on unresolved values;
7. build. Defaults are applied lazily at 15 call sites: `builder/body.rs:34,40,171,234,262`, `builder/mod.rs:276,308`, `builder/contact.rs:28`, `builder/tendon.rs:19`, `builder/sensor.rs:26`, `builder/equality.rs:100,199,259,323,378`;
8. `validate_tendons` after the body tree (`:299`).

### This order produces wrong models today (new findings, measured; none is in the triage)

| case | what | ours (main) | MuJoCo 3.5.0 |
|---|---|---|---|
| f01 | default geom `pos="0.5 0 0"`, geom inside `<frame pos="0 0 1">` | geom_pos (0,0,1): the frame's composed pos blocks the default | (0.5, 0, 1) |
| f02 | class sets `fromto`; geom in a frame | (0.5,0,0): fromto not transformed | (0.5, 0, 1) |
| k01 | `discardvisual` + a class with contype=conaffinity=0 | geom kept (ngeom 2): `compiler.rs:28-29` reads `contype.unwrap_or(1)` | removed (ngeom 1) |
| k02 | `fusestatic` + the fused body's `childclass` | fused geom loses the class (friction 1.0) | keeps it (0.3) |
| c01, c02 | `<composite type="cable">` under root defaults or a body `childclass` | generated geoms and joints take the user's defaults (armature 0.2, friction 0.3) | not applied (0, 1.0) |

- MuJoCo builds composite elements from composite-local defaults (`mjCComposite::SetDefault`, `user_composite.cc:108-125`; `mjs_addGeom(body, &def[0].spec)`, `:392`), never the user's classes.
- **Corpus exposure** (grep): docs combining `discardvisual` or `fusestatic` with `<default>`: 0. Composite docs with `<default>`, childclass or frame: 0 of 24. Frame + default docs: 5, and their defaults set only `contype`, which the frame transform does not touch. So the expected corpus model changes from reordering are **0 by these scans** (not measured by running a branch).

### Options
- **(a) Checks at each `apply_to_*` site.** Rules spread over about 15 call sites in five files. It fixes none of the order bugs above (f01, f02, k01, k02, c01). Duplicate-name and reference checks across element kinds still have no single place.
- **(b) A resolve pass that produces a fully-defaulted `MjcfModel` before frames, composites and compiler passes, then ONE validation pass, then build.** This is MuJoCo's order: the reader applies defaults at element creation, frames and composites exist in the spec, and the compiler checks resolved objects (`checksize` `user_objects.cc:3758`, etc.).
- **(c) Apply defaults eagerly inside the parser**, as MuJoCo's reader does. MuJoCo reads all `<default>` sections first (`xml_native_reader.cc:1017-1022`), so ours would need two passes. `parse_mjcf_str`'s output would no longer be the parse; `MjcfModel` is also the `.mjb` payload (`mjb.rs`). Rejected.

### Recommendation: (b)
```rust
// builder/mod.rs::model_from_mjcf, new order
let mut mjcf = mjcf.clone();
let resolver = DefaultResolver::new(&mjcf.defaults)?;      // S10 structure, L26
resolve::apply_defaults(&mut mjcf, &resolver)?;            // every class / childclass checked; S3 overlay; S11 Options
expand_frames(&mut mjcf.worldbody, &mjcf.compiler);        // composes RESOLVED pos/quat/fromto (f01, f02)
let ex = expand_composites(&mut mjcf.worldbody)?;          // generated elements get MuJoCo's built-ins, never user classes (c01)
mjcf.contact.excludes.extend(ex);
if mjcf.compiler.discardvisual { apply_discardvisual(&mut mjcf); }   // on resolved contype (k01)
if mjcf.compiler.fusestatic    { apply_fusestatic(&mut mjcf); }      // on resolved classes (k02)
validation::validate_resolved(&mjcf)?;                     // ONE place for attribute/name/reference rules
let mut builder = ModelBuilder::new();                     // no `resolver` field; no apply_to_* calls
```
- **`resolve::apply_defaults`.** Walks worldbody (bodies, frames, worldbody geoms and sites with the root only, as `builder/body.rs:22-24` already documents), actuators, tendons, sensors, pairs, equality and meshes.
  - It carries nearest-ancestor `childclass` through bodies **and** frames, as `frame.rs:93-99,152-235` and `body.rs:158-262` do today. That logic moves rather than being rewritten.
  - It writes resolved values and returns `UnknownClass` for any unknown name.
  - Write it **iteratively** (an explicit stack), so mjcf-H2's depth handling has one fewer recursive walk to wrap.
- **`validate_resolved`.** The validation area's rules plug in here (ranges, non-finite, sizes, condim, S12 duplicates, S13 references). Today's `validate()` checks move here too. Running after expansion makes joints inside frames and composite-generated names visible, which is mjcf-S13's blindness. Rules that need built quantities (moving-body mass after inertia, MuJoCo's compiler) stay in the builder where those quantities exist.
- **Public API.**
  - `validate(&MjcfModel) -> Result<ValidationResult>` keeps its signature and runs the same front end (resolve → expand → compiler passes → `validate_resolved`). Its one downstream caller, `sim/L0/tests/integration/site_transmission.rs:786` (expects `Err` on multiple transmission targets), keeps passing.
  - `validate_tendons` folds into it and is un-exported (`lib.rs:206`; 0 uses outside sim-mjcf).
  - `DefaultResolver` is un-exported (§3).
- **Cost.**
  - One new module, about 300–400 lines by estimate (not measured).
  - The 15 `apply_to_*` call sites are deleted, along with `validate_childclass_references` (`frame.rs:32-82`), the childclass plumbing in `expand_frames` and `process_body`, and the duplicate resolver build (`builder/mod.rs:240,265`).
  - Behaviour changes f01, f02, k01, k02, c01 and c02, each a MuJoCo-parity fix with a must-flip doc.
  - Risk: childclass semantics must move exactly. The tests guarding them are `builder/frame.rs` AC20–AC35 and `builder/build.rs:1557-1615` (t16/t17).

### Composite-generated bodies (validating them is new)
- The 24 corpus docs with `<composite>` were checked two ways.
  - **As they are:** 18 load in ours. RULECHK on the built model checks moving-body mass < 1e-15, the inertia triangle, size ≤ 0 for the needed components, and limited lo > hi. RULECHK was made to fail once on `rulechk_neg/neg.xml` (1/1/1/1) and pass on `pos.xml`. Result: 17 clean, and 1 (`263f733b2df7d3fe`, `composite.rs` t1) with `size_nonpos=4`.
  - **With MuJoCo's curve keyword** (`curve="l"` → `"s"`): MuJoCo 3.5.0 loads 11 of 24. It refuses 8 by **its own** 2-body-cable exclude naming ("body 'B_1' not found in bodypair 0"), and 5 by composite attribute rules (`mj_comp.txt`).
- **Our curve keyword map is wrong.** `parser/composite.rs:293-297` maps "s" → Sin, "c" → Cos, "l" → Line. MuJoCo 3.5.0 maps "s" → LINE, "cos(s)" → COS, "sin(s)" → SIN, "0" → ZERO (`xml_native_reader.cc:851-856`; same at 3.4.0 `:817-822`). Ours also truncates more than 3 tokens silently, where MuJoCo errors (`:2565-2566`).
  - Today, `curve="s 0 0"` produces zero-length cable segments (measured above). So **the only planned-rule failure found among composite-generated bodies comes from that keyword bug, not from generation**.
  - In-tree users of `"l"`/`"c"`: 22 corpus docs (26 `curve=` occurrences), plus the composites stress-test validator (`examples/fundamentals/sim-cpu/composites/stress-test/src/main.rs:156,211,280,340-365`). Fixing the map flips them. That is the parser area's report; I only flag the dependency.

---

## 5. sim-mjcf `config.rs` under core-D4 (deleting `SimulationConfig`, `SolverConfig`, `Gravity`)

- **Now.** `config.rs:7` imports all three. It implements `From<&MjcfOption>`/`From<MjcfOption> for SimulationConfig` (`:12-48`) and defines `pub struct ExtendedSolverConfig { pub base: SimulationConfig, … }` (`:54-172`), re-exported at `lib.rs:192`. It has 5 unit tests (`:180-236`).
- **Consumers (git grep):**
  - `ExtendedSolverConfig` has 0 outside `config.rs`/`lib.rs`; only `sim/docs/MUJOCO_GAP_ANALYSIS.md:1206-1210` and `sim/docs/todo/…/SPEC_D*.md` mention it.
  - `From<MjcfOption> for SimulationConfig` has 0 users.
  - `config.rs` is sim-mjcf's **only** use of sim-types (`Cargo.toml:25`; `serde` feature `sim-types/serde` at `:61`).
- **Effect of deleting the three types.** `config.rs` stops compiling. `ExtendedSolverConfig` cannot keep `base`, and without it the struct is a field-by-field copy of `MjcfOption` with no consumer.
- **Recommendation.**
  - Delete `config.rs` and the `pub use config::ExtendedSolverConfig` (`lib.rs:181,192`).
  - Drop the `sim-types` dependency and `sim-types/serde` from sim-mjcf's `Cargo.toml`.
  - The 5 tests go with the file.
  - Update the doc example at `MUJOCO_GAP_ANALYSIS.md:1206-1210`.
  - This must land **in core-D4's commit** (core series), or that commit does not compile.

---

## 6. Commit list (mjcf series, my area), each green on its own

1. **mjcf-E1 + E2** (the first mjcf commit). `error.rs` is rewritten (enum, `ElementKind`, `ValueCount`, `Location`). `ModelConversionError` is deleted and `model_from_mjcf` returns `Result<Model>`. All 334 sites are mapped (§1 table). `Io{path}`. The parse-stage locator in `parse_mjcf_str`, and build-stage `Location::element`.
   - Tests: §1 and §2 adds.
   - Flips: `lib.rs:293-310`; variant renames in `validation.rs`, `lib.rs:281`, `parser/tests.rs:194,1360`, `mjb.rs`, `include.rs`, `error.rs`, `fluid_forces.rs:1197,1217`.
   - Doc: `cf-design/src/mechanism/builder.rs:246`.
   - ⚠ `mjb.rs` tests need a local `--features mjb` run, which CI skips (design verification plan).
2. **Resolve pass + S10 + ledger-L26** (the architecture commit).
   - `DefaultResolver::new -> Result`, with ordered snapshot resolution, made crate-private.
   - Parser: the self-closing nested `<default>`, the actuator-shortcut merge, and top-level `"main"` normalization.
   - `resolve::apply_defaults` and the new order in `model_from_mjcf`. `validate()` moves after expansion; `validate_tendons` folds in.
   - Removal of `validate_childclass_references` and of the builder's `apply_to_*` calls.
   - Must-flip docs: d01–d09, d12, d15, d11, L26 (both shapes), f01, f02, k01, k02, c01, c02.
   - Flips: `default_classes.rs:276`, the 8 `defaults.rs` unit tests.
3. **S11.** The 16 `Option` fields, in parser and resolve pass. Muscle per O-4. Must-flip: the s11_* cases.
4. **S3 + checksize.** `size: Option<Vec<f64>>`, the `overlay` helper, default 0, and `checksize` with MuJoCo's messages. Must-flip: s3_* and ov_class_size_chain. **Depends on the parser area's curve keyword fix landing first**, or `composite.rs` t1 goes red.

### Dependencies on other areas
- **Parser (mjcf-S1, S2, H2):**
  - S1's schema pass owns d10 (`RepeatedElement`) and the allowlist entry for `<default><sensor>`, which S11 keeps. Our `<default><actuator>` child (`parser/defaults.rs:46`) is not in MuJoCo's default schema either (`xml_native_reader.cc:151-206`), so S1 decides it.
  - S2 decides partial arrays: `friction="0.7"`, `fluidcoef` T39, `gear`. The S3 overlay helper defines what a partial array means once accepted.
  - The curve keyword map must precede S3's `checksize`.
  - H2's depth wrapper must cover `parse_mjcf_str`'s locator (iterative already) and the resolve pass (iterative as specified).
- **Validation:** its rules live in `validate_resolved` (commit 2 creates the home). Its variants are `InvalidValue`, `NonFinite`, `InvalidModel`, `DuplicateName` (S12), `UndefinedReference` (S13), and `Unsupported` (ball-then-slide, joints in frames if deferred). The D2 `.mjb` cap may want an `MjbTooLarge` variant; the enum is `#[non_exhaustive]`, but everything in this PR freezes together, so add it now if D2 wants one.
- **Core:** core-D4's commit carries the `config.rs` deletion (§5).

---

## 7. Open questions (the fixed decisions do not settle these)

- **O-1 Actuator default cross-talk (x_crosstalk).** MuJoCo shares ONE actuator default across shortcuts. A `<position kp=10>` default gives a `<velocity>` kv 10 and a `<general>` a position servo (measured). Ours keeps per-shortcut fields.
  - Options: (A) port MuJoCo's representation (`gainprm`/`biasprm`/`dynprm` + `mjs_setTo*` transforms; rewrites `MjcfActuatorDefaults`, large); (B) refuse an element whose class's actuator default was written by a different shortcut kind (stricter; `general` would be a stated limitation); (C) leave it, which the rule does not license.
  - Corpus exposure: **0 docs** (`scripts/actdefscan.py`; its positive control found `x_crosstalk`).
  - **Recommend B for 0.10.** If A is wanted instead, S11's Option fields become the wrong shape for actuators, since they would hold `gainprm`-level values, so decide this before commit 3.
- **O-2 Snapshot semantics (d05).** Recommended: parity, which the ordered resolver gives for free. If Jon prefers live inheritance, which is ours today, that deviation is not licensed by the rule (MuJoCo's result is deterministic and documented by code), and 0 corpus docs have repeated top-level defaults.
- **O-3 `DefaultResolver` public or crate-private.** Recommend private, since re-exporting later is additive. If kept public: `new -> Result`, and `apply_to_*` must return `Result` for unknown classes. Its API is then frozen with no known consumer.
- **O-4 Muscle negatives** (`s11_muscle3_force77`, `m01_muscle_neg`).
  - MuJoCo silently ignores a negative `lmin`/`scale`/`timeconst`/`range`/`fpmax`/…, and gives explicit `force=-1` the class's force instead of "automatic".
  - Under decision 1 ("silently other than the file asked" → refuse): refuse negative muscle parameters except `force`. Refuse `force < 0` only when the resolved class sets `force ≥ 0`.
  - If parity is preferred instead, the `Option` semantics need "negative = None" for muscle fields.
- **O-5 Cylinder `bias`.** MuJoCo 3.5.0 reads 3 values into one `double` (`xml_native_reader.cc:2456`) and uses only `bias[0]`. `bias="1 2 3"` gives MuJoCo biasprm [1,0,0] and ours [1,2,3] (measured, `x_cyl_bias`). Under the rule this is "silently other than asked" → refuse `bias[1..3] ≠ 0`. This belongs to whichever area owns actuator attributes; it is not in my rows.
- **O-6 A dropped vs. refused `curve`.** MuJoCo 3.5.0 refuses its own 2-body cables (exclude names `B_1`, 8 corpus docs once the curve is fixed). Matching that would refuse valid cables. Not my row; flagged for the composite owner.

## 8. New findings not in the triage or ledger (measured)
1. Defaults ignore `<frame>` composition: f01, f02.
2. `discardvisual` and `fusestatic` read unresolved classes: k01, k02.
3. Composite elements take the user's defaults: c01, c02.
4. A self-closing nested `<default class/>` is dropped: d15.
5. Actuator shortcuts in one default replace each other (d11), and cross-shortcut inheritance differs (x_crosstalk).
6. Repeated top-level defaults: MuJoCo snapshots at class creation (d05).
7. The composite `curve` keyword map is wrong. `curve="s"` gives zero-length cable geoms, which the CI test `composite.rs` t1 does not catch because it checks topology only.
8. MuJoCo's own muscle sentinel and cylinder `bias` UB (O-4, O-5).

Repo after: HEAD `3520544e`, `git status --short` empty.
