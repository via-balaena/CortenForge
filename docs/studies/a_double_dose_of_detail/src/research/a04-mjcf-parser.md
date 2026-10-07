> Research for *A Double Dose of Detail*, written during planning by a read-only researcher at `3520544e`. Paths under `$SCRATCH` are the planning session's scratch and are **not in the repo**; every claim here is re-established by the tests its commit adds. Where this section and the book's spec chapters (Parts 0–4) disagree, the spec chapters win.

# Rigid spec — area: sim-mjcf parser strictness

Rows: mjcf-S1 + mjcf-H2 (decisions 19, 16), mjcf-S2 (decision 20), mjcf-S4, mjcf-S5, mjcf-S8,
mjcf-S14, mjcf-S15 + ledger-L28. Parity target MuJoCo **3.5.0** (tag 3.5.0, 881544c); MuJoCo
lines below are `xml_native_reader.cc` (XNR), `xml_util.cc` (XU), `xml_util.h`, `xml_base.cc`,
`user_objects.cc` (UO), `user_mesh.cc`, `user_api.cc` at that tag. Our lines are at
`3520544e`, paths under `sim/L0/mjcf/src/`.

Scratch (everything below is reproducible from it): `SCRATCH/rigid_spec/mjcf_parser/`
(`scripts/`, `data/`, `stackprobe/` — target dir deleted at the end).

---

## 0. Method, and what it cannot see

| what | how | referent |
|---|---|---|
| MuJoCo 3.5.0 schema | extracted `MJCF[]` (XNR:97-528, 245 rows = `nMJCF`, `xml_native_reader.h:105`) with a port of `mjXSchema`'s constructor (XU:324-368); cross-checked against `mujoco.mj_printSchema(False, False)` from the `mujoco==3.5.0` wheel | `scripts/mj_schema.py`, `scripts/parse_printschema.py` → **171 paths, 0 differences** |
| what OUR parser reads | hand transcription of every `get_attribute_opt`/`parse_*_attr`/dispatch arm in `parser/*.rs` | `scripts/ours.py` (each row cites its file:line) |
| per-element diff | `scripts/diff_ours.py` → `data/diff_ours.json` | 35 elements with MuJoCo attrs we never read; 60 elements where we read non-MuJoCo attrs; 37 MuJoCo elements we never dispatch; 105 elements we dispatch that MuJoCo lacks |
| corpus usage | ElementTree walk of the 1,584 static docs, the 157 `format!` templates (placeholders → `1`; 9 do not parse as XML and were not scanned), the 15 repo `.xml`, the 253 submodule `.xml` | `scripts/scan_corpus.py`, `data/{corpus,template,repo,sub}_usage.json` |
| flip prediction | `scripts/strict_sim.py`: a Python MODEL of the proposed parser (schema + allowlist + keywords + numbers + required attrs + orientation + connect form), run over the corpus, joined with our results (`emb_1.jsonl`) and MuJoCo 3.5.0's (`corpus350.jsonl`) | validated against MuJoCo itself in a "no extensions" mode: on the corpus it agrees with 3.5.0's parse-type verdict on every doc except 32 `implicitspringdamper` docs (that keyword was not removed in that mode — a bug in the validation run, not in `rec`), 8 markdown elision/placeholder docs, 2 non-parse MuJoCo errors, and 2 non-finite docs (decision 3) |
| MuJoCo behaviour | `mujoco==3.5.0` `from_xml_string` on ~90 synthetic cases | `scripts/oracle_cases.py`, `oracle_more.py`, `depth350*.py`, `cable350.py` → `data/oracle350_cases.json` |
| OUR behaviour on main | the same cases through `load_model` in a scratch binary | `scripts/ours_cases.py` → `data/ours_cases.json` |
| stack | scratch binary `stackprobe` (path dep on sim-mjcf), binary search of the smallest thread stack that does not overflow, 16 KB resolution, three profiles | `scripts/stackdrive.py`, `data/stack_*.txt` |

**What this method cannot see.** (1) The flip model is a model: rules the corpus never
exercises are untested by the join; the model was validated only against MuJoCo, not against
the Rust implementation (which does not exist yet). (2) Runtime-generated MJCF: 7 generator
sources (`cf-design mechanism/mjcf.rs`, `therm-env builder.rs`, `urdf converter.rs`,
`fsu-model coupled.rs`/`lib.rs`, `cf-codesign`, `cf-mjcf-emit`) were checked by element/attribute
NAME and keyword VALUE grep only, not by pairing attributes to elements. (3) Template keyword
and numeric values are runtime (`{x}` → `1`): template keyword/number findings were discarded.
(4) Submodule files that `<include>` others were scanned per file; included content is only seen
when the included file is itself one of the 253. (5) Which tests ASSERT on a value that a rule
changes (not just "load ok/err") was checked by reading only for the tests named below.
(6) Whether a test is `#[ignore]`d was not checked per site.

Repo: HEAD `3520544e` and `git status --short` empty, before and after (§12).

---

## 1. Shared foundation A — the schema pre-pass (mjcf-S1, mjcf-H2 depth, mjcf-S14)

### 1.1 What MuJoCo 3.5.0 does (the thing we port)

- `mjXReader::Parse` runs `schema.Check(root, 0)` before any reading (XNR:961-973): every
  element and attribute is checked against `MJCF[]`; failure is
  `"Schema violation: unrecognized element"` (XU:503, 554) /
  `"unrecognized attribute: 'x'"` (XU:511) /
  `"unique element 'x' found N times"` for `?` rows (XU:565, 574).
- Recursion: the `body` row is type `R`; `NameMatch` (XU:474-489) lets `worldbody` (level 1),
  `body`, `frame` and `replicate` match it. `Check` descends recursively ONLY into children named
  `body` (XU:517-523); a `<frame>`/`<replicate>` child is accepted by the `missing && !(R && NameMatch)`
  escape (XU:553) **without being checked**. Consequence (measured, 3.5.0): `<frame bogus="1">`,
  `<frame><body bogus="1">`, and `<frame><geom typ="box">` all LOAD (the typo is ignored, geom
  becomes a sphere); `<frame><gemo/>` is refused by the READER, `"unrecognized model element 'gemo'"`
  (XNR:3818). Same as the 3.4.0 DIFFERS item — **re-checked at 3.5.0: holds**.
- The reader adds rules the schema lacks: `World body cannot have attributes` (XNR:3508),
  `... cannot have joints` (3545, 3558), `... cannot have inertia` (3527).
- Depth: no MuJoCo rule; tinyxml2 refuses deep documents with `XML parse error 18`. **Measured
  (3.5.0, identical at 3.4.0)**, depth counted with `<mujoco>` = 1: a self-closing element loads
  at depth 499 and is refused at 500; an element written `<x></x>` (even empty) loads at 498 and is
  refused at 499 (`scripts/depth350b.py`). As chains: 496 nested `<body>` each holding a `<geom/>`
  is the deepest that loads; 497 with the innermost `<body/>` self-closing.
- Includes are expanded before `Parse` (`xml.cc` `IncludeXML`), each included file parsed by its own
  tinyxml2 document — so MuJoCo's depth limit applies PER FILE, not to the spliced tree.

### 1.2 Target

One **iterative** pre-pass over the quick-xml event stream, run by `parse_mjcf_str` before the
existing parser, that refuses:

1. an unknown element or attribute (MuJoCo 3.5.0 table ∪ our allowlist, §3) —
   `UnknownElement` / `UnknownAttribute`;
2. a MuJoCo-valid element or attribute we do not implement (§4) — `UnsupportedElement` /
   `UnsupportedAttribute` (stated limitation);
3. a second `?` child (`DuplicateElement`), e.g. two `<flag>` in one `<option>`, two `<joint>`
   in one `<default>` (both measured: 3.5.0 refuses, main loads);
4. depth beyond MuJoCo's: `Start` at depth ≥ 499 or `Empty` at depth ≥ 500 (`DepthExceeded`);
5. an `<include>` element (S14, §8.5) — `IncludeError`, same variant and text as today;
6. worldbody attributes / `joint` / `freejoint` / `inertial` under `<worldbody>` (MuJoCo reader
   rules XNR:3508/3527/3545/3558; main silently drops them — measured `worldbody_joint`: njnt 0);
7. anything under `<frame>` exactly as under `<body>` — **stricter deviation** (MuJoCo never
   schema-checks a frame's subtree; decided, decision 19). Frame attributes = what MuJoCo's reader
   reads for a frame: `name childclass pos quat axisangle xyaxes zaxis euler` (XNR:3629-3657).
   `joint`/`freejoint`/`inertial` in a frame keep OUR refusal (decision 5 deferred);
8. malformed or duplicate attributes (today `get_attribute_opt` swallows them with
   `.flatten()`, `parser/attrs.rs:33`; 3.5.0 refuses a duplicate with `XML parse error 7`, measured);
9. a second top-level element (**stricter**: 3.5.0 silently uses the FIRST root, measured
   `two_roots`; main uses the LAST, `parser/mod.rs:61-71`).

Keyword values, numbers and required attributes are NOT in the pre-pass — they are the
attribute reader's job (§2), as in MuJoCo (reader, not schema).

### 1.3 Change

New files (all `pub(crate)` unless stated):

```rust
// parser/schema/mujoco_3_5_0.rs — GENERATED, do not edit (header names the generator,
// the wheel version and the date). Mirrors MJCF[] node for node.
pub(crate) enum Repeat { One, AtMostOne, Many, Recursive }        // '!', '?', '*', 'R'
pub(crate) struct SchemaNode {
    pub name: &'static str,
    pub repeat: Repeat,
    pub attrs: &'static [&'static str],      // sorted
    pub children: &'static [SchemaNode],
}
pub(crate) static MUJOCO_3_5_0: SchemaNode = SchemaNode { name: "mujoco", repeat: Repeat::One,
    attrs: &["model"], children: &[ /* 170 nodes */ ] };

// parser/schema/overlay.rs — hand-written, every row cites its reason
pub(crate) enum Verdict {
    /// CortenForge extension, listed in MUJOCO_CONFORMANCE.md "Intentional Divergences".
    Extension,
    /// MuJoCo-valid, not implemented: refused as a stated limitation.
    Unsupported(&'static str),
}
pub(crate) struct OverlayRow { pub path: &'static str, pub attr: Option<&'static str>, pub verdict: Verdict }
pub(crate) static OVERLAY: &[OverlayRow] = &[ /* §3 + §4 */ ];

// parser/schema/mod.rs
/// MuJoCo's tinyxml2 limits, measured on mujoco==3.5.0 (spec §1.1).
pub(crate) const MAX_DEPTH_WITH_CONTENT: usize = 498;
pub(crate) const MAX_DEPTH_SELF_CLOSING: usize = 499;
pub(crate) fn check(xml: &str) -> crate::Result<()>;   // iterative; no recursion
```

Algorithm of `check`: a `Vec<Open>` stack, `Open { node: &'static SchemaNode, kind: Ctx, seen: u64 /*bit per '?' child*/, name: String }`
where `Ctx ∈ {Plain, WorldBody, Body, Frame}`. On `Start`/`Empty`: depth test, root test,
`include` test, child lookup (body-like contexts map `body` → the `body` node, `frame` → the
overlay's frame node, `replicate` → `Unsupported`), `?`-multiplicity bit, attribute loop
(`e.attributes()` WITH checks, errors propagated), push on `Start`. On `End`: pop.
Comments, CDATA, PIs, text: ignored (3.5.0 loads text inside `<body>` and CDATA inside `<worldbody>`, measured).

`parse_mjcf_str` (`parser/mod.rs:50-54`) becomes:

```rust
pub fn parse_mjcf_str(xml: &str) -> Result<MjcfModel> {
    crate::stack::on_large_stack(|| {
        schema::check(xml)?;
        let mut reader = Reader::from_str(xml);
        reader.config_mut().trim_text(true);
        parse_mjcf_reader(&mut reader)
    })
}
```

The parser's `_ => skip_element(..)` / `_ => {}` arms (`mod.rs:174,177-191`, `body.rs:53,69,151,193,571,599`,
`actuator.rs:242-246`, `sensor.rs:151-154`, `asset.rs:40,51`, `contact.rs:33,45`, `equality.rs:63,98`,
`tendon.rs:33-36,109`, `keyframe.rs:31`, `composite.rs:132,146`, `deformable.rs:46,62,147,196,434,454`,
`defaults.rs:73,105`) become **unreachable by construction**; they return an internal error
`MjcfError::XmlParse("schema pre-pass let <path> through")` instead of skipping (never `unreachable!`:
grade Safety). The dispatch `from_str` drops (`actuator.rs:26,35`, `sensor.rs:29,38`, `tendon.rs:30,40`)
likewise.

**How the table stays in sync with 3.5.0.** A golden text file
`sim/L0/mjcf/tests/assets/mujoco_schema_3.5.0.txt` = `mujoco.mj_printSchema(False, False)` from the
pinned wheel (459 lines; regenerated by the same script that writes `mujoco_3_5_0.rs`, kept next to
the conformance generators). A unit test parses the golden text and asserts the const tree equals it,
node for node and attribute for attribute. A MuJoCo bump = rerun the script; the test fails until
the const tree is regenerated. (Verified the round trip in scratch: text ↔ source extraction, 0 diffs.)

**How the overlay stays in sync with the parser.** The failure mode that matters is "table accepts
X, parser ignores X" (silently ignored again). Guard: in `cfg(test)` (or `debug_assertions`) the
attribute reader (§2) records which attribute names it read per element, and `parse_mjcf_reader`
asserts every attribute present on the element was read. Every test that parses MJCF then checks
the sync for the docs it exercises. → Open question Q4.

### 1.4 The depth limit and the stack (mjcf-H2, decision 16)

**Now.** No depth limit. Parsing and building recurse per body/frame/default level
(`parser/body.rs:85` ↔ `:530` mutual recursion, `parser/defaults.rs:21`, `validation.rs:177,287,413`,
`builder/body.rs:94`, `builder/frame.rs:37,61,90,124`, `builder/composite.rs:26,452`,
`builder/compiler.rs:26,40,137`, derived `Clone`/`Drop` of `MjcfBody`).

**Measured stack** (smallest thread stack that does not overflow; `data/stack_*.txt`; macOS arm64):

| entry | doc | opt-level 0 (a downstream crate's default `dev`) | opt-level 2 + debug-asserts (repo `[profile.dev]`/`[profile.test]`, `Cargo.toml:914-918`) | opt-level 3 (`release`) |
|---|---|---|---|---|
| `parse_mjcf_str` | 496 nested bodies (MuJoCo's max) | **20,735 KB** | 3,327 KB | 3,359 KB |
| `load_model` | same | 20,751 KB | 3,343 KB | 3,375 KB |
| `model_from_mjcf` | same, pre-parsed | 2,319 KB | 1,023 KB | 1,055 KB |
| `validate` | same | 1,007 KB | 159 KB | 159 KB |
| `clone`+`drop` | same | 1,983 KB | 671 KB | 623 KB |
| `parse_mjcf_str` | 495 nested frames | 12,591 KB | 2,703 KB | 2,703 KB |
| `model_from_mjcf` | same | 3,455 KB | 1,455 KB | 1,519 KB |
| `parse_mjcf_str` | 497 nested defaults | 18,079 KB | 2,623 KB | 2,607 KB |
| `DefaultResolver::from_model` | same | **79 KB** | 31 KB | 31 KB (31 KB at n=100 too) |
| `load_model` | `cable` composite, count 1000 | not measured (RAM, below) | not completed | 2,095 KB |

Slopes: parse ≈ 41.6 KB/level at opt-0, ≈ 6.6 KB/level at opt-2/3 (n = 100 → 496). A Rust test
thread has 2 MB by default; Windows' main thread 1 MB.

**Correction to the design file.** `DefaultResolver::from_model` does not recurse: `defaults.rs:65-67`
→ `new` → `resolve_all` (loop, `:752-771`) → `resolve_single` (`while`, `:774-803`); measured constant
stack (31 KB at 100 and 497 nested classes). It needs no wrapper. (Its cycle hang, ledger-L26, is the
defaults area's.)

**Depth is not bounded by the XML limit.** `<composite type="cable" count="N">` builds an N-deep body
chain after parsing (`builder/composite.rs:452-462` `append_to_chain`, recursive). MuJoCo 3.5.0 loads
count 1000 (nbody 1000, 20.8 s) and fails count 3000 with `"Caught an unknown exception!"`
(`scripts/cable350.py`; cause not isolated).

**Target.**
1. The pre-pass enforces MuJoCo's depth rule (§1.2.4) — parity.
2. One helper runs every recursive public entry point on a thread with a fixed large stack:

```rust
// src/stack.rs (new, pub(crate))
/// Stack for every recursive entry point. 64 MiB covers the deepest measured case,
/// parsing 496 nested bodies at opt-level 0 (20.7 MiB), with 3x margin; the
/// reservation is virtual memory, committed only as touched.
pub(crate) const LOADER_STACK_BYTES: usize = 64 << 20;
pub(crate) fn on_large_stack<T: Send>(f: impl FnOnce() -> T + Send) -> T;
// std::thread::scope + Builder::new().name("sim-mjcf-loader").stack_size(..).spawn_scoped;
// a panic is re-raised with resume_unwind; on target_family = "wasm" (no threads) f runs inline.
```

   Wrapped: `parse_mjcf_str`, `load_model`, `load_model_from_file`, `model_from_mjcf`, `validate`,
   `validate_tendons` (the last two are `pub` and recurse, `validation.rs:156,407`). `load_model` and
   `load_model_from_file` call private unwrapped inner functions inside ONE thread (one spawn per load).
3. A tree-depth guard for trees that did not come from our XML: an ITERATIVE
   `fn body_tree_depth(&MjcfBody) -> usize` (bodies + frames) checked (a) at entry of
   `model_from_mjcf`, `validate`, `validate_tendons` on the caller's `MjcfModel`, (b) after
   `expand_composites` (`builder/mod.rs:251-256`). Refuse depth > `MAX_BODY_TREE_DEPTH = 4096` with
   `DepthExceeded { limit: 4096 }` — **stricter deviation** (MuJoCo has no stated limit; it loads a
   1000-chain and fails a 3000-chain). 4096 × the largest measured opt-0 builder slope
   (3,455 KB / 495 frames ≈ 7.0 KB) ≈ 28.6 MiB < 64 MiB — **extrapolated, not measured at 4096**.
4. `MjcfModel::body()` / `joint()` (`types.rs:4293-4339`, recursive `find_body`/`find_joint`) become
   iterative worklists. `MjcfModel::all_bodies()` (`types.rs:4276-4289`) **skips every second level**:
   measured on W→A→B→C it returns `["A","C"]` (`stackprobe all_bodies abc`); fix in the same commit
   (0 callers outside its own crate's test names, `git grep "all_bodies()"`).

**Residual risk (stated).** A parsed `MjcfModel` at the XML depth limit is returned to the caller and
DROPPED on the caller's thread; drop recursion was measured only together with clone (1,983 KB at
opt-0). An iterative `Drop for MjcfBody` would remove it but forbids moving fields out of an
`MjcfBody` by destructuring — not checked whether the crate does that. → Q5.

**Observed while measuring, not my area:** the `cable` count-1000 load (repotest profile) had
**1,979,216 KB RSS** in a `ps` sample; I stopped the run (BRIEF RAM rule) — cause not isolated. Count
up to 4096 would be accepted by the guard above; whoever owns composites should look before Rigid ships.

### 1.5 Tests to add (pre-pass + stack)

| test | input | main (measured unless noted) | after |
|---|---|---|---|
| `unknown_attribute_refused` | `<body bogus="1">` | loads (`unknown_attr_body`) | `UnknownAttribute{element:"mujoco/worldbody/body", attribute:"bogus"}` |
| `unknown_element_refused` | `<mujoco><gemo/>…` | loads (`unknown_top`) | `UnknownElement` |
| `typo_attribute_refused` | `<geom typ="box" size="1 1 1"/>` | sphere (user; triage TRUE) | `UnknownAttribute` |
| `frame_subtree_is_checked` (3 cases) | `<frame bogus>`, `<frame><body bogus>`, `<frame><geom typ>` | all load (measured; MuJoCo too) | refused (stricter) |
| `frame_unknown_child_refused` | `<frame><gemo/></frame>` | loads, dropped (measured) | refused (MuJoCo: reader refuses, XNR:3818) |
| `worldbody_joint_refused` | `<worldbody><joint type="free"/>` | njnt 0 (measured) | `World body cannot have joints` |
| `worldbody_attribute_refused` | `<worldbody pos="0 0 1">` | loads (measured) | refused |
| `two_flags_refused` / `two_default_joints_refused` | as named | load (measured) | `DuplicateElement` (3.5.0 measured) |
| `duplicate_attribute_refused` | `<geom size="0.1" size="0.2"/>` | first wins (by reading `attrs.rs:33`) | XML error (3.5.0: `XML parse error 7`) |
| `second_root_refused` | two `<mujoco>` roots | last root used (by reading) | refused |
| `depth_limit_self_closing` | innermost `<body/>` at depth 499 / 500 | no limit | 499 loads, 500 `DepthExceeded` |
| `depth_limit_with_content` | innermost `<body></body>` at depth 498 / 499 | no limit | 498 loads, 499 refused |
| `load_at_depth_limit_from_small_thread` (own test file, so an abort cannot take other tests down) | 496 nested bodies, called from a 256 KB thread | **aborts** (needs 3,343 KB at opt-2, measured) | loads |
| `caller_built_tree_depth_guard` | `MjcfModel` with a 5000-deep chain built in code (iteratively) → `model_from_mjcf` | overflow (by the slopes above; not run) | `DepthExceeded{limit:4096}` |
| `schema_table_matches_mujoco_golden` | golden text vs const tree | n/a | equal |
| `all_bodies_returns_every_level` | W→A→B→C | `["A","C"]` (measured) | `["A","B","C"]` |

---

## 2. Shared foundation B — the attribute reader (mjcf-S2, and the length rules of mjcf-S8 / ledger-L28)

### 2.1 What MuJoCo 3.5.0 does

- Numbers (`ReadAttr` → `ReadAttrVec` → `StringToVector`, XU:700-722, `user_util.cc:1237-1257`):
  C `strtod`/`strtol` per token, separators = C `isspace`, **trimmed at both ends**; a bad token →
  `"bad format in attribute 'x'"` (XU:713); overflow AND underflow → `"number is too large in
  attribute 'x'"` (XU:711; measured `1e999`, `1e-999`, `1e-310`); hex floats accepted (`0x1p1` = 2,
  measured); empty string = attribute absent (measured `mass=""`, `pos=""`, `size=""`, `user=""`).
- Integers (`ReadAttrInt` → `ReadAttrArr` → istringstream, XU:600-626): `"3.0"`, `"1e0"`, an
  out-of-range value and the empty string → `"problem reading attribute 'x'"` (XU:624; measured);
  `" 3 "`, `"+3"` accepted (measured).
- Lengths (`ReadAttr(.., len, .., required, exact)`, XU:798-816; `exact` defaults to `true`,
  `xml_util.h:151-153`): more than `len` → `"attribute 'x' has too much data"` (always); fewer →
  `"does not have enough data"` only when `exact`. With `exact = false` the given PREFIX overwrites
  the element's current value, which was initialised from its default class: **measured**
  `friction="0.7"` → `[0.7, 0.005, 0.0001]`; under `<default><geom friction="1 0.1 0.2"/>` →
  `[0.7, 0.1, 0.2]`; `solref="0.05"` → `[0.05, 1.0]`; `gear="5"` → `[5,0,0,0,0,0]`;
  `fluidcoef="0.5 0.25"` → loads, `[0.5, 0.25, 1.5, 1.0, 1.0]`. The `exact=false` attributes, from the
  ReadAttr call table (`data/` extraction of all 323 `ReadAttr` calls): `size`, `friction`,
  `solref`/`solimp` (+`limit`/`friction`/`fix` variants), `o_solref`/`o_solimp`/`o_friction`, `gear`,
  `gainprm`/`biasprm`/`dynprm`, `fluidcoef`, `polycoef`, `springlength`, composite `count`/`size`,
  sensor `interval`, `actuatorgroupdisable`.
- Quaternions (`ReadQuat`, XU:830-843): exactly 4 values; all-zero → `"zero quaternion is not allowed"`.
- Keywords (`MapValue`, XU:931-948): exact, case-sensitive, untrimmed match against an `mjMap`
  (XNR:517-937) → `"invalid keyword: 'x'"`. Booleans are the keyword map `false|true`
  (`bool_map`); `limited`-style attributes are `false|true|auto` (`TFAuto_map`); flags are
  `disable|enable`.
- Required attributes: `required = true` call sites (XNR, listed in §2.4).

### 2.2 Now — every lenient site, by file (grep at 3520544e; definitions excluded)

| pattern | effect today | sites (file:count) | total |
|---|---|---|---|
| `parse_float_attr` (`attrs.rs:42-44`, `.parse().ok()`) | bad text → attribute silently absent; no trim (`mass=" 1"` ignored, user) | body 13, options 15, actuator 16, sensor 4, defaults 35, contact 2, tendon 7, keyframe 1, composite 9, deformable 2 | 104 |
| `parse_int_attr` (`attrs.rs:47-49`) | same for ints (`condim="3.0"`, `group="99999999999"` silently ignored — measured) | body 7, options 15, actuator 2, sensor 1, defaults 12, asset 1, contact 1, tendon 1, composite 6, deformable 6 | 52 |
| `attr_bool` (`attrs.rs:136-138`, `s == "true"`) | anything but `true` = false (`limited="1"`, `limited="auto"` → false) | body 3, actuator 4, defaults 9, tendon 1 | 17 |
| raw `== "true"` | same | `body.rs:225`; `options.rs:247,276,286,291,296,320,323,326,331`; `equality.rs:136,168,197,226,255`; `deformable.rs:334,340,343` | **18** (the row's "14" was not reproduced) |
| `parse_flag` `v != "disable"` (`options.rs:346-348`) | any value but `disable` = enable (`gravity="off"` keeps gravity) | 25 flags through 1 helper | 1 |
| keyword `from_str` result dropped | unknown keyword silently keeps default (`integrator="RK45"` → Euler) | `options.rs:80,87,124,129`; `defaults.rs:125,164` | 6 |
| case-folded keywords | `"euler"`, `"newton"`, `"RADIAN"` accepted (MuJoCo refuses, measured) | `options.rs:205,252,301`; `types.rs:36,73,106,141` | 7 |
| `parts.len() >= N` then drop / take first N | short → attribute silently absent; long → tail silently dropped | `attrs.rs:152` (`attr_fixed`, 52 callers: body 10, options 3, defaults 20, contact 4, equality 11, tendon 4); `actuator.rs:86,92,143,156,165,205,212`; `defaults.rs:139,228,234,269,276,295,304,334`; `body.rs:290,470`; `composite.rs:177,223,234,245,256`; `deformable.rs:258,267,314,323`; `tendon.rs:147,207` | 28 + 52 |
| `parse_vector3`/`parse_vector4` (`attrs.rs:68-93`, `< N` error, `> N` tail dropped) | `pos="0 0 0 1"` loads (measured); friction `"1 2 3 4"` → (1,2,3) (measured) | direct: body 7, options 3, equality 2, deformable 3; via `attr_vec3`/`attr_vec4`: body 20, defaults 17, asset 1 | 15 + 38 |
| gear `take(6)` (`actuator.rs:73`, `defaults.rs:241`) | 7 values load (measured) | 2 | 2 |
| `if let Ok(parts) = parse_float_array(..)` | unparseable `user`/`range`/`rgba`/`springlength` silently dropped | body 4, actuator 1, sensor 1, defaults 5, tendon 4, deformable 3 | 18 |
| `.parse().unwrap_or(d)` / `.parse().ok()` / `filter_map(parse.ok)` | silent default / silently skipped tokens | deformable 15 + 14 (12 are `filter_map`); `equality.rs:218`; `composite.rs:50` | ≈31 |
| raw attribute bytes (`attrs.rs:35`, no entity unescape) | `name="a&amp;b"` read as `a&amp;b` (3.5.0: `a&b`, measured) | 1 helper, 278 call sites | 1 |

Keyword misreads found in the process:
- **composite `curve`** (`parser/composite.rs:293-297`) maps `s`→Sin, `c`→Cos, `l`→Line, `0`/`zero`→Zero.
  MuJoCo's `shape_map` (XNR:851-856) is `s`→LINE, `cos(s)`→COS, `sin(s)`→SIN, `0`→ZERO. Measured:
  `curve="s 0 0"` (MuJoCo's line) builds 5 bodies all at the origin in ours, spaced 0.25 in 3.5.0;
  `l` is refused by 3.5.0 (`"The curve array contains an invalid shape"`, XNR:2570). More than 3
  tokens: MuJoCo refuses (XNR:2566), ours ignores the rest.
- composite `<joint kind="x">` with `x ≠ main` is silently dropped (`composite.rs:121-124,138-141`);
  3.5.0: `kind` is required and must be `main` (XNR:2642, `jkind_map`).
- `limited="auto"` (and `ctrllimited`/`forcelimited`/`actlimited`, tendon `limited`): MuJoCo keyword
  (TFAuto, XNR:1804,2255,2300-2302); ours reads it as `false`.
- `<default><joint type="bogus">` / `<default><geom type="bogus">` silently dropped (`defaults.rs:125,164`).
- `trimesh|triangle_mesh|nonconvex` geom types (`types.rs:1217`) and `cylindrical|planar` joint types
  (`types.rs:1464-1465`): ours accepts; `cylindrical|planar` then fail in the builder
  (`builder/joint.rs:53-57`); the geom aliases build a plain mesh (`builder/geom.rs:263`). 0 in-tree MJCF
  uses (git grep; the two `type="planar"` hits are URDF).

### 2.3 Target and change

One reader per element replaces the free functions in `parser/attrs.rs`:

```rust
// parser/attrs.rs
pub(super) struct Attrs<'a> {
    element: &'a str,                       // for messages
    list: Vec<(&'a str, Cow<'a, str>)>,     // XML-unescaped; malformed/duplicate attribute -> Err
    #[cfg(test)] read: RefCell<Vec<&'static str>>,   // §1.3 sync guard
}
impl<'a> Attrs<'a> {
    pub(super) fn new(e: &'a BytesStart<'a>) -> Result<Self>;
    pub(super) fn text(&self, name: &'static str) -> Option<&str>;                    // untouched, like ReadAttrTxt
    pub(super) fn real(&self, name: &'static str) -> Result<Option<f64>>;             // 1 number, exact
    pub(super) fn int(&self, name: &'static str) -> Result<Option<i32>>;              // ReadAttrInt rules
    pub(super) fn array<const N: usize>(&self, name: &'static str) -> Result<Option<[f64; N]>>;  // exact N
    pub(super) fn prefix<const N: usize>(&self, name: &'static str) -> Result<Option<Prefix<N>>>; // 1..=N
    pub(super) fn vector(&self, name: &'static str) -> Result<Option<Vec<f64>>>;      // any length
    pub(super) fn quat(&self, name: &'static str) -> Result<Option<Vector4<f64>>>;    // exact 4, non-zero
    pub(super) fn keyword<T: Copy>(&self, name: &'static str, map: &'static [(&'static str, T)]) -> Result<Option<T>>;
    pub(super) fn boolean(&self, name: &'static str) -> Result<Option<bool>>;         // false|true
    pub(super) fn tf_auto(&self, name: &'static str) -> Result<Option<TfAuto>>;       // false|true|auto
}
pub(super) fn required<T>(v: Option<T>, element: &str, attr: &'static str) -> Result<T>;  // "required attribute missing"
```

Number rules (all from §2.1): tokens split on C `isspace` (space `\t \n \v \f \r` — note Rust's
`split_ascii_whitespace` omits `\v`); each token through `f64::from_str`; non-finite → refused
(decision 3; the message for an overflowing literal is MuJoCo's "number is too large"); a token
whose mantissa has a nonzero digit but parses to 0.0 or a subnormal → "number is too large"
(parity with ERANGE underflow, measured `1e-999` and `1e-310`); empty/all-space → `None` for
real/array/prefix/vector, error for `int` (both measured). **Deviation (limitation): hex floats**
(`0x1p1`) are refused — Rust's parser has no hex; 0 uses in corpus, templates, repo and submodule
files (grep `0x[0-9a-fA-F]*p`). → Q6.

New public types (`types.rs`, exported):

```rust
/// A numeric attribute MuJoCo reads with `exact = false` (`xml_util.h:151-153`): the file may give
/// the first k <= N components; components k..N keep the value from the element's default class,
/// else MuJoCo's default. Measured on mujoco==3.5.0: friction="0.7" -> [0.7, 0.005, 0.0001].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Prefix<const N: usize> { values: [f64; N], len: usize }   // invariant: 1 <= len <= N
impl<const N: usize> Prefix<N> {
    pub fn new(given: &[f64]) -> Option<Self>;          // None if empty or > N
    pub fn given(&self) -> &[f64];
    pub fn over(&self, base: [f64; N]) -> [f64; N];     // given components replace base's
    pub fn over_prefix(&self, base: &Prefix<N>) -> Prefix<N>;   // class-chain merge
}
/// MuJoCo's `false | true | auto` keyword (`TFAuto_map`, xml_native_reader.cc:552-556).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TfAuto { False, True, Auto }
```

Field type changes (public structs, not `#[non_exhaustive]` ⇒ breaking, allowed in 0.10;
`git grep` shows no struct-literal construction of these types outside sim-mjcf):
`friction` (geom `Option<Prefix<3>>`, pair `Option<Prefix<5>>`), every `solref*`/`solimp*`
(`Option<Prefix<2>>`/`Option<Prefix<5>>`), `o_solref`/`o_solimp`/`o_friction`, actuator `gear`
(`[f64; 6]` → `Option<Prefix<6>>`), `dynprm` (`Option<Vec<f64>>` → `Option<Prefix<10>>`),
`gainprm`/`biasprm` (`Option<Vec<f64>>` → `Option<Prefix<9>>`: MuJoCo's arrays are 10 long
(`mjmodel.h:40-41`) but sim-core stores 9 (`types/model.rs:663,668`); today a 10th value is silently
truncated by `floats_to_array` (`builder/actuator.rs:722-730`) — refuse 10 values as a stated
limitation, 0 corpus uses (the longest in-tree `gainprm` has 9 values, 5 docs)), `fluidcoef`, tendon `springlength`, equality `polycoef`
(joint: `Prefix<11>` — allowlisted arity, §3; tendon: `Prefix<5>`), geom/site `size` (`Prefix<3>`,
shared with S3), and the matching `Mjcf*Defaults` fields. `limited`, `ctrllimited`, `forcelimited`,
`actlimited` (`Option<bool>` → `Option<TfAuto>`). Merging moves into the resolver: every
`apply_to_*`/`merge_*_defaults` that touches one of these fields uses `over`/`over_prefix`
instead of "child `Some` wins" (`defaults.rs:160-760,805-1075` — the defaults area's file, §10).

Keyword tables: new `parser/keywords.rs` mirroring each `mjMap` used (XNR:517-937), with our
extensions marked in place (§3): `option@integrator` adds `implicitspringdamper`;
`gaintype|biastype|dyntype` add `hillmuscle|millardmuscle` (`builder/actuator.rs:675-722` implements
them); composite `curve` = MuJoCo's map. Joint `cylindrical|planar` and geom
`trimesh|triangle_mesh|nonconvex` are removed (dead aliases). The pub `Mjcf{Integrator,SolverType,
ConeType,JacobianType}::from_str` (`types.rs:35,72,105,140`) become exact-match (0 callers outside
`parser/`, git grep).

### 2.4 Required attributes (3.5.0 reader, `required = true`) that our parser does not require

From the XNR call table: `inertial@pos,mass` (3530, 3532); `<tendon><fixed><joint>@coef` (3988) and
`<spatial><pulley>@divisor` (3982) — ours default 1.0 (`tendon.rs:88,104`); `<spatial><site>@site`,
`<geom>@geom` (3969, 3974) — ours silently drops the element (`tendon.rs:72,94`); composite
`<joint>@kind` (2642); sensor targets: `site` for touch/accelerometer/velocimeter/gyro/force/
torque/magnetometer/rangefinder (4064-4092), `joint`/`tendon`/`actuator` for the joint/tendon/actuator
families (4131-4201), `objtype`+`objname` for the nine frame sensors (4207-4291), `body` for subtree
sensors (4305-4313), user `dim` (4430); `refname` given without `reftype` → error (4214 and peers).
Measured on 3.5.0: missing `coef`, `divisor`, `kind`, `inertial@pos` → `"required attribute missing"`.
Corpus: 0 docs lack `coef`/`divisor`/`kind` (independent ElementTree count: 99 fixed-tendon joints,
4 pulleys, 28 composite joints); the sensor and inertial ones flip docs (§7).

### 2.5 Tests to add (S2)

All run on main through our loader (`data/ours_cases.json`) unless marked "user" (outside user's
probe, verified TRUE by triage) — each must fail on main:
`timestep="0.01s"` (user: stays 0.002) · `damping="abc"` (user: 0) · `mass="2kg"`, `mass="2,5"` (load,
ignored) · `mass=" 2 "` → mass 2 (MuJoCo trims; ours ignores, user) · `condim="3.0"`, `group="99999999999"`
(load) · `pos="0 0 0 1"` (loads) · `friction="1 2 3 4"` (loads as 1,2,3) · `gear` ×7 (loads) ·
`integrator="RK45"` (user: Euler) · `integrator="euler"`, `solver="newton"`, `angle="RADIAN"` (load) ·
`<flag gravity="off"/>`, `<flag gravity="true"/>` (load) · `limited="1"` (loads) · `limited="auto"`
with a class default `limited="false"` and a range → MuJoCo auto (limited when autolimits) ·
`type=" box"`, `type="Box"` (already refused; message becomes MuJoCo's `invalid keyword`) ·
`curve="s 0 0"` body positions (0 vs 0.25 spacing, measured) · `mass="1e-999"` (ours loads with 0;
3.5.0 refuses) · `name="a&amp;b"` resolves as `a&b` · `friction="0.7"` (ours refuses; §8.6) ·
`solref="0.05"` → `[0.05, 1.0]` (ours silently uses the default `[0.02, 1]` — `attr_fixed` returns
`None`, `attrs.rs:152-158`) · `fluidcoef="0.5 0.25"` loads (ours `InvalidFluidCoef`).

---

## 3. The allowlist — every non-MuJoCo thing our parser reads, classified

Classes: **EXT** = real extension we implement, stays (allowlist, documented in
`MUJOCO_CONFORMANCE.md` Intentional Divergences); **SPELL** = a misspelling of a MuJoCo name with the
same meaning → fix the docs; **DEAD** = read by nobody / no effect → refuse, fix the docs.
Counts = corpus docs (templates in brackets) whose parse would flip without the doc fix.

| path · attribute / element | ours | class | uses | remedy |
|---|---|---|---|---|
| `option@integrator=implicitspringdamper` | `types.rs:41` | EXT | 32 | allowlist keyword |
| `general@dyntype/gaintype/biastype=hillmuscle\|millardmuscle` | `builder/actuator.rs:675-722` | EXT | 5 (+ `cf-mjcf-emit/src/lib.rs:277` emits `millardmuscle` at runtime) | allowlist keywords |
| `equality/distance` (name class geom1 geom2 distance solimp solref active) | `parser/equality.rs:203-229`, `builder/equality.rs:59` | EXT (3.5.0 schema has no `distance`, XNR:355-371) | 12 (8 examples) | allowlist element |
| `equality/joint@polycoef` with 6–11 values | `builder/equality.rs:253-256`; sim-core degree ≤ 10 (`constraint/equality.rs:212-218,265`) | EXT (arity) | 1 corpus + 8 in `cf-mjcf-emit/tests/assets/knee_ref.xml` (9 values, generated) | allowlist arity 11 |
| `deformable/flex/pin` (id) | `deformable.rs:137-146,187-195` | EXT | 21 | allowlist |
| `deformable/flex@mass` | `deformable.rs:246-248` | EXT | 2 | allowlist |
| `deformable/flex/elasticity@bending_model` | `deformable.rs:362-367` | EXT | 3 [+1] | allowlist (keyword `cotangent\|bridson` exact; today anything else silently = cotangent) |
| `deformable/flexcomp` + children contact/elasticity/edge/pin | `deformable.rs:384-494` | EXT (MuJoCo's `flexcomp` lives under `<body>`, XNR:312-329; ours is a different, smaller generator under `<deformable>`) | 3 | allowlist; `type` other than `grid\|box` → refuse (today: silently empty flex, `:399`) — see Q3 |
| `default/sensor` (noise cutoff user) + `sensor@class` | `defaults.rs:406-417`, `sensor.rs:63` | EXT if decision 4 keeps sensor defaults | 2 | allowlist (contingent) |
| `muscle@actearly` | `actuator.rs:209` | EXT? (MuJoCo: `actearly` only on `general`/`plugin`) | [1] `activation_clamping.rs:518` | Q2 |
| `deformable/flex/vertex@pos`, `deformable/flex/element@data` (child elements) | `deformable.rs:103-136,158-186` | SPELL — MuJoCo: `<flex vertex="…" element="…">` attributes (XNR:332-333), which ours does NOT read | 74 [+7] | implement the attribute form; rewrite docs; refuse the child form — Q1 |
| frame sensors `site=` / `body=` / `geom=` | `sensor.rs:66-72` + inference `builder/sensor.rs:298-300` (site → body as **xbody** → geom) | SPELL — MuJoCo: `objtype` + `objname`, both required (XNR:4207-4291) | 38 (incl. 1 example) | rewrite `site="s"`→`objtype="site" objname="s"`, `body="b"`→`objtype="xbody" objname="b"`, `geom="g"`→`objtype="geom" objname="g"`; pre-registered check: model fingerprint unchanged per rewritten doc |
| `flag@*="true"` | `options.rs:347` | SPELL (`enable`) | 1 test + 1 md | rewrite |
| `option@integrator="euler"` | `types.rs:36` | SPELL (`Euler`) | 1 | rewrite |
| composite `curve` `l` | `composite.rs:296` | SPELL (`s`) | 16 (`l 0 0` 14, `l 0 s` 2) [+1] | `l`→`s`; `l 0 s` → `s 0 sin(s)` (2 examples, keeps their geometry); `composite.rs:28` `s 0 0` changes meaning (sine→line), its asserts are structural only (`:37-75`, read) |
| `default/motor@kp,kv` | `defaults.rs:246-247` (one struct for all actuator defaults) | SPELL (belongs on `position`) | 1 (`parser/tests.rs:795`) | rewrite to `<position>` |
| `deformable/flex@density`, `flexcomp@density` | **not read anywhere** (only `body.rs:381`, `defaults.rs:167`, `composite.rs:275`, `options.rs:153` read `density`) | DEAD (pinned as ignored: `parser/tests.rs:2026-2050` "dt16 … silently ignored") | 72 [+7] | remove; dt16 test flips to "refused" |
| `freejoint@damping` | not read (`body.rs:329-335` reads only `name`) | DEAD | 3 examples | remove (behaviour unchanged) — or honour intent with `<joint type="free" damping>`, which CHANGES those validators → Q7 |
| `compiler@exactmeshinertia` | `options.rs:330-336` (warns "no effect") | DEAD (3.5.0 schema lacks it) | 3 [+10: `exactmeshinertia.rs` ×9, `mesh_inertia_modes.rs:322`] | remove; the `exactmeshinertia.rs` tests become refusal tests |
| `adhesion@gear` | read, no effect for body transmission (pinned: `adhesion.rs:260-300` "ac6 gear has no effect") | DEAD (3.5.0 adhesion has no `gear`, XNR:434-435) | 2 | ac6 becomes a refusal test |
| `option@nconmax,njmax` | parsed (`options.rs:161-166`), never read by the builder (git grep) | DEAD (MuJoCo has them on `<size>`) | 1 | remove |
| `option@regularization,friction_smoothing` | builder reads them (`builder/mod.rs:784`, `build.rs:439-440`) | EXT unused in-tree (0 docs) | 0 | decision 19 allowlists only what in-tree code uses → refuse; Model fields stay settable in code |
| `flex/edge@solref`, `flex@selfcollide` | not read on those elements | DEAD | 1 + 1 | remove |
| `touch@objtype`; `framelinacc/frameangacc@reftype,refname` | parsed into fields the builder ignores for these types | DEAD | 1 + 1 (both `sensor_phase6.rs`, tests pinning "ignored") | flip to refusal tests |
| `flag@passive` | not read (comment `options.rs:356-357`) | DEAD | 1 (`runtime_flags.rs:1385`, pins "ignored") | flip |
| `<actuator/*/plugin>`, `<sensor/*/plugin>` child, and all plugin/`<extension>` MJCF | parsed (`actuator.rs:226-260`, `sensor.rs:135-168`, `extension.rs`) but the builder sets `nplugin: 0` and every `*_plugin` to `None` (`builder/build.rs:491-497`) | DEAD today; depends on core decision 9 | 2 corpus + the 13 `<extension>`/`<plugin>` uses in `parser/tests.rs` (t1–t13) | refuse as `Unsupported` until a builder wires plugins |
| `default/actuator`, `deformable/skin/vertex`, `skin@mesh` | read | not used in-tree (0) | 0 | refuse |
| `tendon/fixed@width,material,rgba`; `fixed/{site,geom,pulley}` children | read (shared code with spatial) | not MuJoCo, 0 uses | 0 | refuse |
| every other B-list attribute (actuator/sensor common superset, e.g. `<touch joint=…>`) | `actuator.rs:53-223`, `sensor.rs:56-132` read one superset for all types | not MuJoCo for that element, 0 uses | 0 | refused by the per-element MuJoCo rows |

With this allowlist the corpus model gives **171 docs** that parse today and would be refused
before the doc fixes (A 74 flex children, B 38 frame sensors, C 85 dead attributes, D 19 keyword
spelling, E 19 too-much-data, F 1, G 6 rule refusals; a doc can be in several). If every
in-tree-used extension were allowlisted instead (`allow_all_ext`): 132. Of the 171, **one** is a doc
MuJoCo 3.5.0 loads (`parser/tests.rs:553`, `<compiler><lengthrange/>` — a stated limitation); every
other newly refused doc is also refused by 3.5.0.

---

## 4. MuJoCo-valid things we never read → stated limitations (decision 19)

Computed as MuJoCo 3.5.0's schema minus `scripts/ours.py` (`data/diff_ours.json`); corpus / submodule
file counts from `data/{corpus,sub}_usage.json`.

**Implemented by this area instead of refused** (MuJoCo semantics cited): body `xyaxes`/`zaxis`
(S8, §8.4); connect `site1`/`site2` (L28, §8.6); `<flex vertex element>` attributes (Q1);
`<actuator><intvelocity>` (S1, below); user sensor `dim` with `dim="1"` only (MuJoCo requires it,
XNR:4430; ours fixes dim 1, `types.rs:3419`; `dim ≠ 1` refused); `needstage="acc"`/`datatype="real"`
accepted (MuJoCo's defaults, `user_init.c:354-355`, equal ours `builder/sensor.rs:575`), other values refused.

**Refused (limitation)** — attributes: actuator `actdim` (general), `tausmooth` (muscle),
`inheritrange` (position, intvelocity; submodule 16+7 files), `cranklength` on default actuators;
joint `springdamper` (sub 7), `actuatorfrclimited`, `actuatorfrcrange` (sub 11+6); freejoint
`align`, `group`; geom `fitscale`; inertial `axisangle xyaxes zaxis euler`; site `fromto` (sub 4);
mesh `builtin class content_type material normal params refpos refquat smoothnormal texcoord`
(sub: `class` 10, `content_type` 3, `refquat` 1); hfield `content_type`; compiler
`inertiagrouprange saveinertial`; flex `elemtexcoord flatskin material rgba texcoord`, elasticity
`elastic2d`; skin `file group texcoord`, bone `vertid vertweight`; weld `site1 site2 torquescale`;
rangefinder `camera data`; tendon `armature actuatorfrclimited actuatorfrcrange`; composite joint
`axis margin solimpfix solreffix solimplimit solreflimit solimpfriction solreffriction type`;
`<size>` `memory njmax nconmax nkey nstack nuserdata nuser_cam` (submodule files: njmax 10, nconmax 10, nkey 9, memory 1, nuserdata 1).
Elements: the **12 sensors** — verified at 3.5.0 in the schema (XNR:446-503) against our
`MjcfSensorType::from_str` (`types.rs:3287-3327`, 37 names): `camprojection, tendonactuatorfrc,
jointlimitpos, jointlimitvel, tendonlimitpos, tendonlimitvel, insidesite, contact, e_potential,
e_kinetic, tactile, plugin`; measured 3.5.0 loads `jointlimitpos` and `e_potential` (nsensordata 1
each), main drops them (nsensor 0 — shifts every later sensordata index); `<actuator><plugin>`;
`<body>` `camera light attach replicate flexcomp`; `<composite>` `skin site plugin`; `<geom><plugin>`,
`<mesh><plugin>`; `<compiler><lengthrange>`; `<custom>`; `<statistic>` (its `meaninertia`
overrides the solver's tolerance scale, `user_model.cc:5143`, `engine_solver.c:326,550,1863`; sim-core
recomputes it per step, `constraint/mod.rs:66-77`); `<asset>` `texture material skin model`;
`<default>` `material camera light`; `<visual>`; `<equality>` `flex flexvert`.

**Corpus impact of the limitations: 3 docs** — `<compiler><lengthrange/>` (`parser/tests.rs:553`); user sensors without `dim` (`callbacks.rs:199-212`, 3 sensors, `dim` is REQUIRED by 3.5.0, fix: add `dim="1"`); `<user dim="3">` (`parser/tests.rs:2283`, refused as dim ≠ 1, and its `<plugin>` child is dead). **Submodule
impact** (not in CI, ledger-L28): of the 53 submodule files that load today, **47 use a visual-only
element** (`camera`, `light`, `material`, `texture`, `visual`, asset `skin`) and 16 use nothing else
unread; 226 of all 253 use one. Refusing visual-only elements makes essentially every real MuJoCo model
fail → **Q8 (the one open question with real weight here).**

---

## 5. mjcf-S1 — unknown attributes/elements; `intvelocity`; the 12 sensors

**Now.** Attributes are looked up by name and never checked (`attrs.rs:32-39`); unknown elements are
skipped (`mod.rs:174`, `body.rs:53,69,151,193`, `actuator.rs:26-46`, `sensor.rs:29-43`). Measured on
main: `<body bogus>` loads; `<gemo/>` vanishes; `<intvelocity joint kp actrange>` → nu 0 (3.5.0:
nu 1, na 1, dyntype integrator, gaintype fixed, biastype affine, gainprm[0] 10, biasprm[1] −10,
actlimited 1, measured); `jointlimitpos`, `e_potential` → nsensor 0.

**Target.** §1 pre-pass + §3 allowlist + §4 limitations. `intvelocity` implemented as MuJoCo's
`mjs_setToIntVelocity` (`user_api.cc:942-954` = `mjs_setToPosition` `:903-937` + `dyntype =
integrator`, `actlimited = 1`; read at XNR:2399-2432).

**Change.** `MjcfActuatorType::IntVelocity` (pub enum, not `non_exhaustive` ⇒ breaking; 0 matches
outside sim-mjcf, git grep); `from_str` arm (`types.rs:2630-2641`); builder arms in
`builder/actuator.rs:32-83` (ctrl/dyn) and `:431-560` (gain/bias); attributes per XNR:412-417 (kp, kv,
dampratio, actrange, `inheritrange` → refused, §4). Prior spec: `sim/docs/todo/future_work_10b.md:88`
(DT-123).

**Tests to add.** the measured cases above, each failing on main; `intvelocity` model fields equal
the 3.5.0 values listed.

**Flips.** §7 classes A–G.

**Downstream.** `MjcfError` gains variants (errors area). No downstream crate calls a parser internal.

---

## 6. mjcf-H2 — see §1.4 (depth limit, stack thread, tree guard, iterative getters).

---

## 7. Corpus flip list (recommended allowlist; `data/flip_table_rec.txt` has every site)

CI status by path: `sim/**/tests/**`, `**/src/**` tests = CI-run tests; `examples/**` = validate-examples;
`*.md` = never run. Docs counted once per rule.

| rule → remedy | sites |
|---|---|
| A flex `<vertex>`/`<element>` children (74) → attribute form | `sim/L0/tests/integration/flex_unified.rs` ×51–55, `flex_flex_collision.rs` ×12, `parser/tests.rs` ×4, `deformable_friction_dt25.rs` ×3, `runtime_flags.rs` ×1; templates `flex_unified.rs:2625,2945,3192,3248,3299,3401`, `flex_flex_collision.rs:29` |
| B frame-sensor spelling (38) | `sensor_phase6.rs` ×15, `sensor_phase6_spec_d.rs` ×6, `sensors_phase4.rs` ×9, `spatial_transport.rs`, `mjcf_sensors.rs`, `builder/compiler.rs`, example `sensors/frame-pos-quat/src/main.rs:51`; plus `sensor_phase6.rs:17-40` pins `objtype == None` for `fp_without_objtype` (flips: objtype required) |
| C dead attrs (85) | flex `density` (as A) · `freejoint damping`: examples `equality-constraints/{connect-to-world,connect-body-to-body,stress-test}` · `exactmeshinertia.rs` ×3 (+9 templates) · `adhesion.rs` ×2 · `flex_unified.rs` (edge solref, flexcomp density) · `flex_flex_collision.rs` (selfcollide) · `parser/tests.rs` (option nconmax/njmax; general/user plugin) · `sensor_phase6.rs` (touch objtype; acc reftype/refname) · `runtime_flags.rs:1385` (flag passive) |
| D keywords (19) | curve `l`: `composite.rs` ×7, `parser/tests.rs` ×2, `implicit_integration.rs`, examples `composites/{stress-test ×3, hanging-cable, cable-catenary, cable-loaded}`; flag `true`: `runtime_flags.rs:2551`, `MUJOCO_GAP_ANALYSIS.md:1209`; `euler`: `implicit_integration.rs:371` |
| E too much data (19) | geom `friction` 5 values: `phase8_spec_b_qcqp.rs` ×10, `unified_solvers.rs` ×9 (+ template `:1467`) — rewrite to 3 values, behaviour unchanged (ours used the first 3) |
| F | `parser/tests.rs:795` default motor kp/kv |
| G | connect without anchor: `equality_constraints.rs` ×2 (+ parse-only `parser/tests.rs:1237,1276,1305,1324,1378` and `equality_constraints.rs:746`, which parse today and fail the build); `inertial` without pos: example `urdf-loading/stress-test/src/main.rs`; multiple orientation: `builder/body.rs:702`; user sensor `dim`: `callbacks.rs:199`, `parser/tests.rs:2274`; `<lengthrange/>`: `parser/tests.rs:553` |

Also flipping, outside the doc corpus (tests that match a variant or pin a lenient value):
`lib.rs:281` (`UnknownJointType` → becomes `InvalidKeyword` if the errors area merges keyword
variants), `fluid_forces.rs:1197` (`InvalidFluidShape`, same), `fluid_forces.rs:1205-1220` t39
(2-value `fluidcoef` now LOADS — 3.5.0 measured), `parser/tests.rs:2026-2050` dt16, `parser/tests.rs`
t1–t13 plugin tests. Parse-ok/build-err docs whose FIRST error changes (message-asserting tests may
flip; not read one by one): `composite.rs:385,395,409,423,437,281` (curve `l` fires first),
`actuator_phase5.rs:1003` (`interp="spline"` now refused in the parser instead of the builder).
err→ok candidates in my rules: `parser/tests.rs:1256` (self-closing body, S4), `fluid_forces.rs:1209` (t39).

Generated MJCF (§0 limit 2): no generator emits a refused name or keyword by grep;
`cf-mjcf-emit` emits 9-value joint `polycoef` (allowlisted arity) and `millardmuscle` (allowlisted).

---

## 8. Items

### 8.1 mjcf-S2 — see §2.

### 8.2 mjcf-S4 — Start/Empty asymmetry (self-closing elements dropped)

**Now** (by reading; S4's mocap case measured by the user, triage TRUE). The `Event::Empty` branches
lack arms the `Event::Start` branches have: `<body/>` under worldbody (`body.rs:56-70`), body
(`:154-194`), frame (`:574-600`); `<composite/>` under worldbody and body; nested `<default class="x"/>`
(`defaults.rs:76-106`) — a class MuJoCo defines even when it has no children (`mjXReader::Default`
adds the class before reading children, XNR:2884-2905); top-level `<default/>` (`mod.rs:177-191`); `<flex …/>` (`deformable.rs:51-64`) — MuJoCo's
usual spelling of a flex is self-closing; `<extension><plugin/>` (`extension.rs:25`, Start only);
`<config …></config>` (`extension.rs:100`, Empty only). Corpus: only `<worldbody><body …/>` occurs
(10 docs: `parser/tests.rs:1237,1256,1276,1305,1324,1378,2194,2327` + 2), scanned by
`scripts/scan_selfclose.py`.

**Target.** Each dispatcher matches `Start | Empty` once and passes `empty: bool` to the child parser,
which reads children only when `!empty`. This removes the class instead of adding ten arms.

**Change.** `parse_worldbody`, `parse_body`, `parse_frame`, `parse_default`, `parse_mujoco`,
`parse_deformable`, `parse_extension`, `parse_plugin_config`, `parse_composite` (private; signatures
gain `empty: bool`). The existing `parse_*_attrs`/`parse_*` pairs collapse.

**Tests to add.** `<body name="t" mocap="true" pos="0 0 1"/>` → nbody 2, nmocap 1 (main: 1, 0 — user);
`<default><default class="e"/></default>` + `<geom class="e">` resolves (main: class missing);
`<flex name="f" dim="2" body="…" vertex="…" element="…"/>` builds (main: dropped — needs Q1's
attribute form).

**Flips.** `parser/tests.rs:1256` and the 7 other self-closing-body docs change from build-err to
build-ok or to a different error (connect rule); all are parse-only or already expected-error tests
(read: 1237-1262).

**Dependency.** Must land no later than the defaults area's undefined-class refusal (S10), or an
empty self-closing class referenced by `class=` would flip there (0 such docs in the corpus).

### 8.3 mjcf-S5 — repeated `<compiler>`/`<option>`

**Now.** `model.option = parse_option(..)` / `model.compiler = parse_compiler_attrs(..)` replace the
whole struct (`mod.rs:87-92,179-182`); the flag likewise (`options.rs:57`). User + triage: TRUE.

**Target.** Each section merges onto the current value, attribute by attribute — MuJoCo loops over
every `<compiler>`/`<option>` calling `Compiler`/`Option` on the same spec (XNR:990-998), each reading
only the attributes present. Measured 3.5.0: `<compiler angle="radian"/><compiler autolimits="true"/>`
→ range `[-1, 1]` radians; `<option timestep="0.001"/><option><flag energy="enable"/></option>` →
timestep 0.001. Two `<flag>` in ONE option → `DuplicateElement` (pre-pass, §1.2.3); one flag in each of
two options → merge.

**Change.** `fn parse_compiler_attrs(e, &mut MjcfCompiler) -> Result<()>`,
`fn parse_option_attrs(e, &mut MjcfOption) -> Result<()>`, `fn parse_flag_attrs(e, &mut MjcfFlag) -> Result<()>`
(private). `parse_size_attrs` already merges.

**Tests to add.** the two measured cases (main: angle back to degree; timestep back to 0.002 — user).

**Flips.** none in the corpus (only `parser/tests.rs:2125` has two `<compiler>`; both set `angle`).

### 8.4 mjcf-S8 — geometry attributes

| sub-item | now | target (3.5.0) | change |
|---|---|---|---|
| exact lengths | §2.2 | §2.1 | §2.3 reader |
| multiple orientation specifiers | builder picks euler > axisangle > xyaxes > zaxis > quat (`builder/orientation.rs:55-60`); measured `euler`+`zaxis` on a body loads with the euler | `"multiple orientation specifiers are not allowed"` (`xml_base.cc:54-77`; measured on geom and body); `quat` counts; inertial: `fullinertia` with any orientation → `"fullinertia and inertial orientation cannot both be specified"` (XNR:3536-3538, measured) | counted in the readers of geom, site, body, frame, inertial |
| zero quaternion | `quat_from_wxyz` normalises → NaN (`orientation.rs:13-15`). **Measured: `<geom quat="0 0 0 0">` makes `load_model` not return within 20 s (killed)** — the mjcf-H1 hang reached through a quaternion, not a NaN literal | `"zero quaternion is not allowed"` (XU:830-843; measured geom, body) | `Attrs::quat` |
| box/ellipsoid `fromto` | `compute_fromto_pose` returns `(size[0], half_length, 0)` for every type (`builder/geom.rs:817-856`); measured box/ellipsoid `size="0.1 0.2" fromto="0 0 0 0 0 1"` → `(0.1, 0.5, 0)` | `(size[0], size[0], half_length)` (UO:3702-3706); **measured `(0.1, 0.1, 0.5)` at 3.5.0 and 3.4.0** — the user's "(0.1, 0.2, 0.5)" is not what either version gives. Also UO:3679-3700: `fromto` only on capsule/cylinder/ellipsoid/box; `pos` with `fromto` → error; points closer than mjEPS → error | `compute_fromto_pose(fromto, size, geom_type)` |
| plane size | `geom_size_to_vec3`'s `_` arm gives `(0.1, 0.1, 0.1)` for every plane (`builder/geom.rs:883`); measured for no size and `size="2 3"` | size as given (prefix over default 0); `size[2] <= 0` → `"plane size(3) must be positive"` (UO:150-156; measured for no size, `"2 3"`, `"1"`) | `Plane` arm returns the given size; the positivity check belongs with the validation area's S7 |
| body `xyaxes`/`zaxis` | never parsed (`body.rs:207-246`); measured identity | measured: `xyaxes="0 1 0 -1 0 0"` → `(0.7071, 0, 0, 0.7071)`; `zaxis="1 0 0"` → `(0.7071, 0, 0.7071, 0)` | `MjcfBody { pub xyaxes: Option<[f64; 6]>, pub zaxis: Option<Vector3<f64>>, .. }` (breaking: new pub fields; no struct literal outside sim-mjcf, git grep); pass them at the 3 body `resolve_orientation` calls that pass `None, None` today: `builder/body.rs:141-148`, `builder/frame.rs:237`, `builder/compiler.rs:152-159` |
| 5-value `fromto`, 4-value `pos` | ignored / 4th dropped (measured) | `does not have enough data` / `has too much data` (measured) | reader |

**Corpus effect.** Plane: all 283 plane geoms in the corpus give 3 values with `size[2] > 0`
(`scripts/scan_planes.py`) → no refusal; `geom_size` changes in 282 docs (expected-change list; whether
any test asserts a plane's `geom_size` was not checked). fromto box/ellipsoid: not counted. Multiple
orientation: 1 (`builder/body.rs:702`).

**Tests to add.** each measured case in the table (main values given there). The zero-quaternion test
must not be run against main (hangs).

### 8.5 mjcf-S14 — `<include` detection

**Now.** `load_model` refuses any string containing `<include` (`builder/mod.rs:367-372`); measured: a
comment `<!-- <include file="x.xml"/> -->` → `IncludeError`. 3.5.0 loads it (measured).
`parse_mjcf_str` itself silently SKIPS a real `<include>` element (`mod.rs:174`).

**Target.** The pre-pass refuses an `include` ELEMENT (event-based: comments, CDATA, attribute values
and text never match) with the same variant and text. `load_model` drops its substring test.
`include.rs:37`'s `contains("<include")` is only a fast path before an event loop (comments are passed
through, `:121-126`) — unchanged.

**Tests to add.** include-in-comment loads (main: refused, measured); `<include>` through
`parse_mjcf_str` → `IncludeError` (main: silently skipped, by reading). Kept passing:
`builder/mod.rs:1121` `test_load_model_string_rejects_includes` (asserts the message mentions include).

**Deviation (stricter, listed).** Depth is counted on the spliced document; MuJoCo counts per file
(§1.1). Not measured whether any in-tree include tree comes near 498.

### 8.6 mjcf-S15 + ledger-L28 — mesh names, single-value friction, connect forms

| sub-item | now | target (3.5.0) | change |
|---|---|---|---|
| unnamed mesh | name `""` (`asset.rs:90`; by reading) | name = file attribute with directory (last `/` or `\`) and LAST extension stripped (`user_mesh.cc:298-302,337-341`, `user_util.cc:768-829`); no name and no file → `"empty name in mesh"` (measured) | in `parse_mesh_attrs`; same for `<hfield>` (UO:4364-4369), whose `name` ours REQUIRES today (`asset.rs:141-143`) |
| `friction="0.7"` | parse error `expected 3 values` (measured) | `[0.7, default[1], default[2]]` from the class chain (measured both with and without a class) | `Prefix<3>` (§2.3) — and every other `exact=false` attribute with it: submodule files use 3-value `solimp` (53 files) and `solimplimit` (24), which ours SILENTLY DROPS today (`attr_fixed`) |
| connect `site1`/`site2` | `body1` required (`equality.rs:120-121`); measured: site form refused | XNR:2113-2145: site form or body form, mixing → `"body and site semantics cannot be mixed"`; body form needs `body1` AND `anchor` → else `"either both body1 and anchor must be defined, or both site1 and site2 must be defined"` (both measured) | `MjcfConnect { body1: Option<String>, site1: Option<String>, site2: Option<String>, anchor: Option<Vector3<f64>>, .. }` (breaking); builder lowers the site form to bodies: `obj1 = site1's body`, `data[0..3] = site1 pos in its body`, `obj2 = site2's body`, `data[3..6] = site2 pos in its body` — and must NOT recompute `data[3..6]` from qpos0 (today's body-form code, `builder/equality.rs:113-120`). Exact at runtime: MuJoCo's site form uses `site_xpos` (engine_core_constraint.c:438-443) = body pose × site pos, the body form uses `xmat·data + xpos` (`:430-435`) |
| weld `site1`/`site2`, `torquescale` | not read | 3.5.0 loads a site weld (measured); body1 without anchor is fine for weld (measured) | refuse as limitation (0 corpus/submodule uses of weld sites measured) → Q9 |

**Tests to add.** each measured case (main results in the table).

**Flips.** connect without anchor (§7 G). Submodule (not CI): 6 files use connect `site1/site2`; files
CONTAINING a short friction (measured, not "first error" as ledger-L28 counted): geom 1-value 32, geom
2-value 3, pair 2-value 1.

---

## 9. Commit list (each compiles and is green alone; each carries the doc fixes and tests it flips)

Assumes the errors area's `MjcfError` commit (E1) is in, and that `Prefix<N>`/`TfAuto` land with or
before the defaults area's S11 (§10).

1. **`stack.rs` + depth-safe traversals (H2 part 1).** `on_large_stack` around the 6 entry points;
   iterative `MjcfModel::body/joint`; `all_bodies` fix; tree-depth guard (`DepthExceeded{4096}`). No
   behaviour change except the guard and `all_bodies`. Tests: §1.5 `load_at_depth_limit_from_small_thread`,
   `caller_built_tree_depth_guard`, `all_bodies_returns_every_level`.
2. **Attribute reader (S2 numbers + required).** `Attrs`; every `parse_*` ported; `Prefix<N>` fields
   (+ resolver merge for those fields); exact lengths; trim; underflow; empty-string rules;
   entity unescape; required attributes of §2.4 EXCEPT the sensor targets and user `dim`;
   `quat` (zero) and multiple-orientation refusal (S8).
   Doc fixes: 19 five-value friction rewrites; `inertial` without `pos` (example
   `urdf-loading/stress-test`); the multiple-orientation site (`builder/body.rs:702`).
   Includes ledger-L28's friction. (Frame-sensor `objtype`/`objname` REQUIRED moves to commit 7 with
   the class-B rewrites; user `dim` REQUIRED to commit 6, where `dim` starts being read.)
3. **Keywords exact (S2 keywords).** `keywords.rs`; `TfAuto`; bool/enable maps; composite curve map
   fixed; allowlisted keyword extensions; dead joint/geom aliases removed. Doc fixes: class D (19).
4. **Start/Empty unification (S4).**
5. **Merge repeated sections (S5).**
6. **MuJoCo forms we implement.** `<flex vertex element>` attributes; `intvelocity`; user sensor
   `dim`; connect `site1/site2` + connect form rule (S15/L28); mesh/hfield names from the file stem
   (S15); body `xyaxes/zaxis`, box/ellipsoid `fromto`, plane size (S8). Doc fixes: class G connect
   (`equality_constraints.rs` ×2 + the 6 parse-only connect tests); `callbacks.rs:199-212` gains
   `dim="1"`; `parser/tests.rs:2283` (`dim="3"`) becomes a refusal test.
7. **Schema pre-pass (S1 + H2 depth + S14).** Generated `mujoco_3_5_0.rs` + golden text + sync test;
   overlay (§3, §4); worldbody and frame rules; include detection; parser `_` arms become internal
   errors; read-tracking sync guard. Doc fixes: classes A, B, C, F; refusal tests for the 12 sensors
   and the limitations. Corpus re-run must show: new refusals ⊆ 3.5.0's refusals ∪ the listed
   deviations (`compiler/lengthrange` only, in the current corpus).

Order rationale: 6 before 7 so the docs can be rewritten to MuJoCo forms before the pre-pass refuses
our spellings; 2–3 before 7 so the pre-pass commit does not also carry value-level flips.

## 10. Dependencies on other areas

- **Errors (E1, E2).** Variants I need (all `String` payloads — attribute and element names come from
  the document): `UnknownElement { path }`, `UnknownAttribute { element, attribute }`,
  `UnsupportedElement { path, reason: &'static str }`, `UnsupportedAttribute { element, attribute,
  reason }`, `DuplicateElement { path, count }`, `DepthExceeded { limit }`, `InvalidKeyword { element,
  attribute, value }`, `AttributeLength { element, attribute, expected: usize, got: usize, exact: bool }`
  (or two variants: too much / not enough), `BadNumber { element, attribute, value }` (MuJoCo's three
  messages: bad format / problem reading / number is too large), `MissingAttribute` with
  `element: String` (exists; `attribute: &'static str` is enough). Whether `UnknownJointType`,
  `UnknownGeomType`, `InvalidFluidShape`, `InvalidFluidCoef` survive (2 tests match them, §7) is the
  errors area's call. E2: the pre-pass and `Attrs` can supply quick-xml's byte position; turning it
  into a line number is E2's.
- **Defaults (S3, S10, S11, ledger-L26).** S11 turns fields into `Option<T>`; for the `exact=false`
  fields T must be `Prefix<N>` and for `*limited` `TfAuto` — define both once, with or before S11.
  S3's default geom `size` should be `Prefix<3>`. S4's `<default class="x"/>` arm must be in before
  S10 refuses undefined classes. `<default><sensor>`'s allowlist row is decision 4.
- **Validation (S7).** Plane `size[2] > 0`, `fromto` point distance, `fromto`+`pos` — the checks live
  wherever the validation area puts post-default checks; the SIZING is mine (§8.4).
- **Core decision 9 (plugins).** Plugin MJCF stays refused until a builder wires plugins.
- **mjcf-H1 owners.** Zero quaternion is a further way into the `from_matrix` hang (§8.4); my reader
  closes that door at parse time, but a class default can still carry orientation values — not traced.

## 11. Open questions (only what the fixed decisions leave open)

- **Q1 flex `<vertex>`/`<element>` children (74 docs + 7 templates).** (a) implement MuJoCo's
  attributes and rewrite the docs, refuse the children [recommended: same meaning, MuJoCo spelling,
  mechanical rewrite]; (b) allowlist the children as well. If (a) is wrong: 81 doc rewrites were churn;
  if (b) is wrong: two spellings of one feature stay documented forever.
- **Q2 `muscle@actearly` (1 template).** allowlist vs rewrite as `<general dyntype="muscle" …
  actearly>`. Recommend allowlist (implemented, 1 use). Wrong ⇒ one more divergence row.
- **Q3 `<deformable><flexcomp>`.** allowlist under its MuJoCo name [recommended, 3 docs] vs rename to
  avoid colliding with MuJoCo's body-level `flexcomp`. Wrong ⇒ users reading MuJoCo docs try
  `<flexcomp type="mesh">` and get a refusal naming a MuJoCo feature.
- **Q4 sync guard between overlay and parser.** read-tracking assertion in `cfg(test)` [recommended]
  vs none. Wrong (none) ⇒ a future parser edit can make an accepted attribute silently ignored again —
  the exact defect this row fixes.
- **Q5 iterative `Drop` for `MjcfBody`.** add (removes the caller-thread drop recursion; check for
  destructuring moves first) vs leave (residual risk at the XML depth limit, measured 1,983 KB
  clone+drop at opt-0). Recommend measure drop alone first; add if > 512 KB.
- **Q6 hex floats.** refuse [recommended, 0 uses anywhere scanned] vs implement C `strtod` hex.
- **Q7 `freejoint damping` in 3 validator examples.** remove (behaviour unchanged) [recommended] vs
  `<joint type="free" damping>` (honours intent, changes the validators' trajectories).
- **Q8 visual-only and capacity-hint MuJoCo elements** (`camera light material texture visual skin`,
  `<statistic>` minus `meaninertia`, `<size memory njmax nconmax nkey nstack nuserdata>`). Decision 19
  read literally refuses them: 0 corpus docs flip, but 47 of the 53 submodule files that load today
  stop loading (226 of 253 use one). Option: a third verdict "accepted, ignored (no effect on any
  computed quantity)" for elements whose only effect is rendering or allocation capacity, each listed.
  I do not recommend either: it is the meaning of "implement" in decision 19, which is Jon's.
  What differs: real-model compatibility (ledger-L28's goal) vs a strictly smaller accepted set.
- **Q9 weld site form.** refuse [recommended now: 0 uses measured] vs implement like connect (the
  orientation part needs `site_quat`, engine_core_constraint.c:501-510 — not worked through).

## 12. Checked, and how — and what was not

Checked: the schema extraction twice (source vs wheel output, 0 diffs); the flip model against
MuJoCo's own verdicts on the corpus (agreement listed in §0); every MuJoCo behaviour quoted as
"measured" was run on `mujoco==3.5.0` (and the depth and fromto ones on 3.4.0 too); every "main"
behaviour marked measured was run through `load_model` at 3520544e; the scan of required attributes
was repeated with an independent ElementTree count; the first scan MISSED the 19 five-value
frictions (the arity gate skipped `friction`) and was caught by cross-checking a known count —
the same class of miss may exist for attributes I did not cross-check. Not checked: Rust code (none
written); opt-0 stack for composites (RAM); whether tests assert plane `geom_size`; message-text
assertions in the 83 parse-ok/build-err docs; `#[ignore]` status of flipping tests.

Hazards hit during the work (both stopped by `timeout`, no process left — `pgrep` empty):
`mass="1e999"` (→ inf) and `<geom quat="0 0 0 0">` both hang main's `load_model`.

### Appendix — every flipping site (corpus docs that parse today; `T:` = format! templates)

Class letters as in §3/§7 (A flex children · B frame-sensor spelling · C dead attribute · D keyword · E too much data · F wrong element · G rule refusal). A site in several classes is listed under the combination.

- **A**: `sim/L0/mjcf/src/parser/tests.rs`:2054,2078,2102; `sim/L0/tests/integration/flex_unified.rs`:203
- **AC**: `sim/L0/mjcf/src/parser/tests.rs`:2034; `sim/L0/tests/integration/deformable_friction_dt25.rs`:22,50,76; `sim/L0/tests/integration/flex_flex_collision.rs`:136,194,255,310,345,391,463,520,590,630,675,759; `sim/L0/tests/integration/flex_unified.rs`:18,38,62,91,113,134,152,169,219,242,1056,1091,1142,1202,1249,1307,1383,1429,1479,1514,1552,1595,1638,1673,1709,1729,1753,1778,1806,1835,1877,1901,1924,1957,1994,2027,2071,2088,2104,2120,2138,2475,2514,2578,2701,2735,2809,2870,2892,2912,3045,3079,3108,3350; `sim/L0/tests/integration/runtime_flags.rs`:2159
- **B**: `examples/fundamentals/sim-cpu/sensors/frame-pos-quat/src/main.rs`:51; `sim/L0/mjcf/src/builder/compiler.rs`:557; `sim/L0/tests/integration/mjcf_sensors.rs`:51; `sim/L0/tests/integration/sensor_phase6.rs`:17,48,88,281,510,743,793,845,874,900,988,1028,1062,1194,1281,1319,1359,1456,1498; `sim/L0/tests/integration/sensor_phase6_spec_d.rs`:63,265,327,481,508,544; `sim/L0/tests/integration/sensors_phase4.rs`:209,243,502,851,887,924,966,1005; `sim/L0/tests/integration/spatial_transport.rs`:67
- **BC**: `sim/L0/tests/integration/sensor_phase6.rs`:1393
- **C**: `examples/fundamentals/sim-cpu/equality-constraints/connect-body-to-body/src/main.rs`:46; `examples/fundamentals/sim-cpu/equality-constraints/connect-to-world/src/main.rs`:46; `examples/fundamentals/sim-cpu/equality-constraints/stress-test/src/main.rs`:388; `sim/L0/mjcf/src/parser/tests.rs`:215,2249; `sim/L0/tests/integration/adhesion.rs`:267,283; `sim/L0/tests/integration/exactmeshinertia.rs`:57,75,355; `sim/L0/tests/integration/flex_unified.rs`:185; `sim/L0/tests/integration/runtime_flags.rs`:1385; `sim/L0/tests/integration/sensor_phase6.rs`:658
- **CG**: `sim/L0/mjcf/src/parser/tests.rs`:2274
- **D**: `examples/fundamentals/sim-cpu/composites/cable-catenary/src/main.rs`:58; `examples/fundamentals/sim-cpu/composites/cable-loaded/src/main.rs`:43; `examples/fundamentals/sim-cpu/composites/hanging-cable/src/main.rs`:44; `examples/fundamentals/sim-cpu/composites/stress-test/src/main.rs`:150,334,351; `sim/L0/mjcf/src/parser/tests.rs`:2143,2168; `sim/L0/tests/integration/composite.rs`:84,115,199,226,253,301,333; `sim/L0/tests/integration/implicit_integration.rs`:371,1202; `sim/L0/tests/integration/runtime_flags.rs`:2551; `sim/docs/MUJOCO_GAP_ANALYSIS.md`:1209
- **E**: `sim/L0/tests/integration/phase8_spec_b_qcqp.rs`:52,89,202,246,364,404,444,477,528,565; `sim/L0/tests/integration/unified_solvers.rs`:1268,1405,1533,1580,1618,1663,1704,1817,1894
- **F**: `sim/L0/mjcf/src/parser/tests.rs`:795
- **G**: `examples/fundamentals/sim-cpu/urdf-loading/stress-test/src/main.rs`:120; `sim/L0/mjcf/src/builder/body.rs`:702; `sim/L0/mjcf/src/parser/tests.rs`:553; `sim/L0/tests/integration/callbacks.rs`:199; `sim/L0/tests/integration/equality_constraints.rs`:504,572
- **T:AC**: `sim/L0/tests/integration/flex_flex_collision.rs`:29; `sim/L0/tests/integration/flex_unified.rs`:2625,2945,3192,3248,3299,3401
- **T:C**: `sim/L0/tests/integration/exactmeshinertia.rs`:104,158,217,267,285,403,420,498,566; `sim/L0/tests/integration/mesh_inertia_modes.rs`:322
- **T:D**: `examples/fundamentals/sim-cpu/composites/stress-test/src/main.rs`:205
- **T:E**: `sim/L0/tests/integration/unified_solvers.rs`:1467 (5-value geom friction)
- **T (Q2)**: `sim/L0/tests/integration/activation_clamping.rs`:518 (`muscle@actearly` — flips only if Q2 is answered "rewrite")
