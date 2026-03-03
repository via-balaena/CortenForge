# Spec C — Parsing Breadth: Spec Quality Rubric

Grades the Spec C spec on 10 criteria. Target: A+ on every criterion
before implementation begins. A+ means "an implementer could build this
without asking a single clarifying question — and the result would produce
numerically identical output to MuJoCo."

**MuJoCo conformance is the cardinal goal.** Every criterion in this rubric
ultimately serves conformance. P1 (MuJoCo Reference Fidelity) is the most
important criterion — grade it first and hardest. A spec that scores A+ on
all other criteria but has P1 wrong is worse than useless: it would produce
a clean, well-tested implementation of the wrong behavior.

> **Split mandate:** §55 (user data) is fully conformance-testable against
> MuJoCo 3.5.0. DT-88 (flexcomp attributes) is **not** — `<flexcomp>` is
> not recognized by MuJoCo 3.5.0's XML parser (see Scope Adjustment / EGT-3).
> DT-88 is verified against MuJoCo documentation and the dm_control schema
> only. P1 grades §55 at full A+ conformance standard; DT-88 at documentation
> fidelity standard.

Grade scale: A+ (exemplary) / A (solid) / B (gaps) / C (insufficient).
Anything below B does not ship.

---

## Scope Adjustment

Spec C covers §55 and DT-88 per the umbrella. Empirical verification
revealed critical facts about both tasks:

| Umbrella claim | MuJoCo 3.5.0 reality | Action |
|----------------|----------------------|--------|
| §55: Per-element `*_user` arrays — parse `user` from body/geom/joint/tendon/actuator/sensor/site; parse `nuser_*` from `<size>` | **Confirmed fully.** MuJoCo 3.5.0 supports user data on all 7 element types. Auto-sizing when no `nuser_*` set (max length across type). Zero-padding for shorter arrays. Error on too-long data when `nuser_*` explicit. Default class inheritance confirmed. `<size>` element with `nuser_body` through `nuser_sensor` attributes confirmed. | **In scope — full conformance** |
| §55 umbrella says 7 types | MuJoCo also supports `cam_user`/`nuser_cam` (camera user data). The umbrella and session plan explicitly list 7 types only. | **Camera out of scope** — track as DT-123 if needed |
| §55 umbrella says `Vec<Vec<f64>>` storage | MuJoCo uses flat storage `(N × nuser_*)` with stride. Umbrella chose `Vec<Vec<f64>>` for Rust clarity. This is conformance-equivalent when all inner vecs have length `nuser_*`. | **In scope — conformance-equivalent** (flat optimization deferred) |
| DT-88: `<flexcomp>` attributes (`inertiabox`, `scale`, `quat`, `file`) | **`<flexcomp>` is NOT recognized by MuJoCo 3.5.0 XML parser.** Testing with `from_xml_string` produces `Schema violation: unrecognized element`. `MjsFlex` spec API has no `inertiabox`, `scale`, `quat`, or `file` attributes. The `<flexcomp>` element is documented at mujoco.readthedocs.io but absent from the 3.5.0 binary. dm_control schema.xml (which CortenForge's parser is based on) includes it. | **In scope — documentation fidelity only** (cannot empirically verify against MuJoCo binary) |
| DT-88: `inertiabox` type | dm_control schema says `float` (scalar). MuJoCo docs page says `real(3)` (3-vector). Cannot verify empirically. | **Use dm_control schema** (`float` = scalar) — CortenForge parser already targets dm_control schema |
| DT-88: `file` attribute runtime loading | Umbrella says "parse and store path". Actual mesh/grid file loading deferred per umbrella Out of Scope. | **Parse + store path only** — file loading deferred |

**Final scope:**

1. **§55:** Per-element user data arrays — parse `user` attribute from 7 element
   types (body, geom, joint, site, tendon, actuator, sensor), parse `nuser_*`
   from `<size>` element, store `*_user` arrays and `nuser_*` fields on Model,
   wire through builder. Default class inheritance for user data. Full MuJoCo
   conformance.
2. **DT-88:** Flexcomp attributes — parse `inertiabox` (f64), `scale`
   (Vec3), `quat` (Vec4), `file` (String) from `<flexcomp>` element. Store on
   `MjcfFlex` or intermediate builder state. Apply scale/quat transforms to
   generated vertex positions during flexcomp expansion. Documentation-fidelity
   only (no MuJoCo binary verification).

---

## Empirical Ground Truth

All §55 findings verified empirically against MuJoCo Python 3.5.0 (`mujoco`
package). DT-88 findings from dm_control schema.xml and MuJoCo documentation
only (binary does not support `<flexcomp>`).

### EGT-1: User Data Auto-Sizing Behavior

**MuJoCo 3.5.0**, Python `mujoco.MjModel.from_xml_string()`.

When no `<size nuser_*="...">` is specified, MuJoCo auto-computes `nuser_*`
as the maximum user array length across ALL elements of that type:

```python
# Model with bodies having different user data lengths:
#   b1: user="1 2 3" (3 values)
#   b2: user="4 5"   (2 values)
#   world: no user   (0 values)
# Result:
#   nuser_body = 3  (max length)
#   body_user shape = (3, 3)
#   body_user[0] = [0, 0, 0]        # world — zero-padded
#   body_user[1] = [1, 2, 3]        # b1 — full
#   body_user[2] = [4, 5, 0]        # b2 — zero-padded to 3
```

When NO element of a type has user data AND no `nuser_*` is set:
```python
# No user data anywhere:
#   nuser_body = 0
#   body_user shape = (N, 0)  — array exists but with zero width
```

When `nuser_*` is explicitly set in `<size>`:
```python
# <size nuser_body="5"/>
# b1: user="1.5 2.5"
# Result:
#   nuser_body = 5
#   body_user[0] = [0, 0, 0, 0, 0]       # world
#   body_user[1] = [1.5, 2.5, 0, 0, 0]   # b1 — padded to 5
#   body_user[2] = [0, 0, 0, 0, 0]        # b2 — no user attr
```

### EGT-2: User Data Error on Too-Long Arrays

**MuJoCo 3.5.0**: When `nuser_*` is explicitly set and an element's user
array is longer, MuJoCo raises an error:

```python
# <size nuser_geom="2"/>
# <geom user="10 20 30 40"/>
# ERROR: "user has more values than nuser_geom in geom 'g1' (id = 0)"
```

This is a validation rule the spec must implement: user data length ≤ nuser_*
when nuser_* is explicitly set.

### EGT-3: User Data Default Class Inheritance

**MuJoCo 3.5.0**: User data IS inherited through default classes — but
**only for element types that support defaults**. Empirically verified:

- **Supports `<default><type user="..."/>`:** joint, geom, site, tendon,
  actuator (via `<general>`/`<muscle>`/etc.), camera (out of scope)
- **Does NOT support defaults:** body (`<default><body>` → schema violation),
  sensor (`<default><sensor>` → schema violation)

```python
# <default>
#   <joint user="100 200"/>
#   <geom user="5 6 7"/>
# </default>
# j1: no user attr → gets [100, 200] from default
# j2: user="300"   → gets [300, 0]  (overrides, padded to nuser_jnt=2)
# g1: no user attr → gets [5, 6, 7] from default
# g2: user="8 9 10"→ gets [8, 9, 10] (overrides)
```

Key: when overriding with fewer values than the default, the element's
values are zero-padded to `nuser_*`, NOT filled with remaining default values.

**Body user data:** parsed directly from `<body user="...">` only. No
default class inheritance. World body cannot have user data in MJCF
(it's implicit, not an XML element).

**Sensor user data:** parsed directly from `<sensor_type user="...">` only.
CortenForge has `MjcfSensorDefaults.user` (types.rs:815) and
`apply_to_sensor()` handles it (defaults.rs:660) — this is a CortenForge
extension (or matches a non-XML MuJoCo API). Spec C does NOT modify the
existing sensor defaults behavior.

### EGT-3a: User Data Stress-Test Edge Cases

**MuJoCo 3.5.0**, additional edge cases discovered during stress testing:

**Validation happens AFTER defaults application:** When `nuser_jnt=2` is
explicitly set in `<size>` and a default class has `user="1 2 3"` (3 values),
MuJoCo raises an error when an element *inherits* the too-long default:
```python
# <size nuser_jnt="2"/>
# <default><joint user="1 2 3"/></default>
# <joint name="j1" type="hinge"/>  ← inherits default, gets 3 values
# ERROR: "user has more values than nuser_jnt in joint"
```
The default itself is parsed without error; the validation triggers at
element compilation time when effective user length > nuser_*.

**`nuser_*=-1` in `<size>` = auto-sizing:** Setting `nuser_body="-1"` is
accepted and triggers auto-sizing (same as omitting the attribute). The
compiled nuser_body ends up as 0 when no body has user data.

**`nuser_*=0` with user data = error:** Setting `nuser_body="0"` while
`<body user="1 2 3">` exists produces an error.

**Auto-sizing includes defaults lengths:** When default has `user="1 2 3 4 5"`
(5 values) and one element overrides with `user="10"` (1 value),
`nuser_geom=5` (max of 5 from default, not 1 from override). The override
element gets `[10, 0, 0, 0, 0]`.

**Override is full replacement, not partial merge:** Element `user="10"`
with default `user="100 200 300"` produces `[10, 0, 0]`, NOT `[10, 200, 300]`.
The element's user data completely replaces the default; remaining positions
are zero-padded to `nuser_*`.

**`user=""` (empty string) = attribute not specified:** Treated identically
to omitting the `user` attribute entirely — meaning defaults cascade still
applies. If a default class provides `user="1 2 3"` and the element has
`user=""`, the element receives `[1, 2, 3]` from the default (NOT empty).
Only when there are no defaults and no other element has user data does this
result in nuser_*=0.

**Nested default classes:** Child default class overrides parent. Element
using child class gets child's user data; element without class gets
top-level default.

**Special float values accepted:** Negative values, extreme magnitudes
(1e300, -1e-300), and zero are all valid user data values.

### EGT-3b: User Data Stress-Test Edge Cases — Round 2

**MuJoCo 3.5.0**, additional edge cases from second stress-test round:

**`nuser_*` strict integer parsing:** Only exact decimal integers accepted.
`"2.5"`, `"3.7"`, `"1.0"`, `"1e2"` ALL rejected with "problem reading
attribute 'nuser_*'". MuJoCo uses strict `atoi`-style parsing, not
`atof` + truncation.

**`nuser_*` validation range:** Only `-1` is valid negative (auto-size
sentinel). `-2`, `-100` etc. error with "nuser_* must be >= -1". No upper
bound enforced — `nuser_body="10000000"` allocates 150+ MB without complaint.

**Multiple `<size>` elements merge:** Legal. Attributes merge with
last-writer-wins per attribute. `<size nuser_body="3" nuser_geom="2"/>`
followed by `<size nuser_jnt="5"/>` produces all three values. If both set
`nuser_body`, the second wins.

**XML order of `<size>` vs `<default>` irrelevant:** MuJoCo does a two-pass
parse: collect all declarations, then validate. Explicit `nuser_*` too small
for any user data (from defaults or inline) is a hard error regardless of
which appears first in the XML.

**Actuator subtype defaults share storage (last-write-wins):** Within a
single default class, `<motor>`, `<position>`, `<velocity>`, `<cylinder>`,
`<muscle>`, and `<general>` are ALL aliases for the SAME underlying default
element. The last one written in XML order wins for ALL subtypes. Example:
```python
# <default>
#   <general user="100"/>
#   <motor user="200"/>     ← overwrites general's value
# </default>
# Result: ALL actuator subtypes get user=[200.]
#
# Reversed order:
#   <motor user="200"/>
#   <general user="100"/>   ← overwrites motor's value
# Result: ALL actuator subtypes get user=[100.]
```
To differentiate subtypes, separate default classes are required.

**`childclass` propagation:** `childclass="foo"` on a `<body>` propagates
default class "foo" to ALL descendant elements (joints, geoms, sites, etc.)
of that body and its sub-bodies, including grandchildren. An explicit
`class=` on an element overrides the inherited childclass.

**`nuserdata` is completely separate from `nuser_*`:** `nuserdata` controls
`MjData.userdata` — a global runtime scratch array. `nuser_*` controls
per-element `user` arrays on `MjModel`. No interaction between the two
systems. Default `nuserdata` is 0 when not specified.

**Default class naming constraints:** Class names are globally unique
regardless of nesting depth. The name `"main"` is reserved by the implicit
root default class. Empty string class name `""` is rejected.

**Whitespace in user data values:** Leading/trailing whitespace, multiple
spaces, tabs (`&#9;`), and newlines (`&#10;`) are all accepted as separators.
MuJoCo uses standard XML whitespace normalization.

**User data float parsing:** Scientific notation (`1e5`, `-2.5e-3`, `0.0e0`)
accepted in user data values. Integer-written values (`1 2 3`) stored as
`float64`. Contrast: `nuser_*` attributes require strict integers.

**`<size>` only valid as direct child of `<mujoco>`:** Placing `<size>`
inside `<worldbody>` or elsewhere produces "Schema violation: unrecognized
element". Empty `<size/>` is a no-op (identical to no `<size>` at all).

**`<flex>` element works in MuJoCo 3.5.0:** Accepts visual/geometry
attributes (`radius`, `rgba`, `material`, `flatskin`, `group`, `texcoord`)
but requires `body` attribute. Physics attributes (`damping`, `stiffness`,
`solref`, etc.) rejected. Only `<flexcomp>` is absent from 3.5.0, not
`<flex>` itself.

### EGT-3c: User Data Stress-Test Edge Cases — Round 3

**MuJoCo 3.5.0**, additional edge cases from third stress-test round:

**`<freejoint>` does NOT accept `user`:** The `<freejoint>` shorthand has
a reduced attribute set. `<freejoint user="1 2"/>` → schema violation. To
set user data on a free joint, use `<joint type="free" user="1 2"/>` instead.
The resulting joint shares `nuser_jnt` with all other joints.

**NaN and Inf accepted in user data:** `user="nan inf -inf"` and
`user="NaN INF -INF"` are both accepted (case-insensitive). MuJoCo emits
a warning ("XML contains a 'NaN'. Please check it carefully.") but does
not reject the model. Values stored as IEEE 754 NaN/Inf.

**Only whitespace separators valid:** Commas, semicolons, pipes all produce
"bad format in attribute 'user'". Even `user="1, 2, 3"` (comma+space) fails
because `"1,"` is not a valid float token. Only space and tab are accepted
as value separators (newlines also work via XML entity `&#10;`).

**`user="0"` vs `user=""`:** `user="0"` produces a 1-element array `[0.0]`
with nuser_*=1. `user=""` or `user="   "` (whitespace only) produces a
zero-length array with nuser_*=0. This is a critical semantic distinction:
the value 0.0 is data; an empty string is absence of data.

**`<key>` does NOT accept `user`:** Keyframes store state snapshots only
(`key_time`, `key_qpos`, `key_qvel`, `key_act`, `key_ctrl`). No per-element
user data in keyframes.

**`<default>` element does NOT accept `user`:** `<default user="1 2 3">`
and `<default class="foo" user="1 2 3">` both produce schema violation.
User data can only be set on child elements within `<default>` (e.g.,
`<joint user="..."/>`, `<geom user="..."/>`).

**`<composite>` element mostly deprecated in 3.5.0:** `grid`, `particle`,
`cloth`, `rope`, `loop` produce deprecation warnings. Sub-elements
(`<geom>`, `<joint>`, `<body>` inside `<composite>`) do NOT accept `user`.

**Body ordering is depth-first pre-order:** body_user[0] is always world,
then depth-first traversal of the body tree. Siblings appear in XML order.
This determines `*_user` array indexing for all element types.

**Sensor ordering is XML document order:** sensor_user indices follow
declaration order in the XML, NOT sorted by sensor type.

**childclass nearest-ancestor rule:** When Body1 has `childclass="A"` and
child Body2 has `childclass="B"`, elements in Body2 (and Body2's descendants)
use class B, not class A. The nearest ancestor with `childclass` wins.

**Saved XML round-trip behavior:** `mj_saveLastXML` produces XML with:
(1) explicit `<size nuser_*="N"/>` for all non-zero nuser values,
(2) user data zero-padded to nuser_* length (e.g., `user="5 6 7 0"` when
nuser=4 but only 3 values specified), (3) all actuator subtypes normalized
to `<general>`. When all nuser_*=0, the `<size>` element is omitted.

**User data physics inertness verified empirically:** Two identical models —
one with user data on all element types, one without — produce bit-identical
`qpos` and `qvel` after 100 simulation steps with identical controls.

**Leading zeros in user values silently stripped:** `user="007 008.5 0000.1"`
parses to `[7.0, 8.5, 0.1]`. No warning.

**No practical limit on user array length:** 5000+ values accepted in a
single user attribute without error.

### EGT-3d: User Data Stress-Test Edge Cases — Round 4

**MuJoCo 3.5.0**, additional edge cases from fourth stress-test round:

**Transitive default class inheritance:** If class A sets `joint user="1 2 3"`,
class B (child of A) sets NO joint user, and class C (child of B) sets NO
joint user, then an element using class C gets `[1, 2, 3]` from A. The chain
propagates through any number of intermediate classes that don't override.

**`childclass` does NOT shadow root default:** If root default has
`joint user="1 2 3"` and class "myclass" (child of root) defines NO joint
user, elements under `childclass="myclass"` STILL get `[1, 2, 3]`. The class
inherits from its ancestors including root. `childclass` selects a class;
it does not block inheritance.

**`class="main"` references root default:** An element can use `class="main"`
to explicitly select the root default class, overriding any `childclass` on
its parent body. This is useful to "escape" from a childclass scope.

**`nuser_*` maximum value:** Strict `< 2147483647` (INT_MAX). The max
accepted value is 2147483646. Value 2147483647 itself produces "nuser_body
is too large. Expected < 2147483647." This implies `int` (i32) storage with
a reserved sentinel.

**`nuser_*` read-only after compilation:** `m.nuser_body = 5` raises
"property of 'MjModel' object has no setter". All nuser_* fields are
read-only on the compiled model.

**XML round-trip precision loss:** `mj_saveLastXML` writes user data with
only ~6 significant digits. A value `1.2345678901234567` round-trips as
`1.23457`. In-memory storage has full float64 precision, but XML
serialization is lossy. No option to control save precision.

**Hex notation accepted in user data:** C-style hex integers (`0xFF` →
255.0) and hex float notation (`0x1p10` → 1024.0) are accepted via
`strtod`. Python-style octal (`0o77`) and binary (`0b1010`) are rejected.

**Denormal/subnormal float values rejected:** Values near `DBL_MIN`
(2.2250738585072014e-308) may be rejected depending on notation. Full
`DBL_MIN` with sufficient significant digits works, but `1e-308`, `5e-324`,
etc. produce "number is too large" (misleading error). Values above
`~2.226e-308` with enough precision are safe.

**+0 and -0 preserved:** `user="+0 -0"` produces `[0.0, -0.0]` with sign
bit preserved (copysign distinguishes them). IEEE 754 comparison treats
them as equal.

**`nuser_*` set with zero elements of that type:** `nuser_sensor="5"` with
zero sensors produces array shape `(0, 5)` — zero rows, nuser columns. The
nuser value persists regardless of whether elements of that type exist.

**Auto-sizing is demand-driven, not declaration-driven:** Auto-sizing
computes `nuser_*` from the maximum **resolved** user array length across
all **instantiated** elements of that type. A default class with
`geom user="1 2 3 4 5"` contributes to auto-sizing ONLY if at least one
geom actually inherits from that class. If no geom references the class
(or all geoms have explicit `user=` that overrides it), the default's
5-value length is ignored. Example: root default has `geom user="1 2 3 4 5"`
and two geoms with no explicit user → `nuser_geom=5` (both inherit).
But if a named class "unused" has `geom user="1 2 3 4 5 6 7 8 9 10"` and
no geom references it, `nuser_geom` is NOT 10.

**Exhaustive element type verification:** Tested ALL non-user element types.
NONE accept `user`: `<inertial>`, `<light>`, `<freejoint>`, `<pair>`,
`<exclude>`, `<numeric>`, `<tuple>`, `<text>`, `<key>`, `<material>`,
`<texture>`, `<mesh>`, `<hfield>`, `<flex>`, `<connect>`, `<weld>`,
equality `<joint>`, equality `<tendon>`. Confirms the exhaustive list is
exactly 8 element types.

**MjSpec API context:** Programmatic model building uses `.userdata`
attribute (type `MjDoubleVec`) on body/geom/joint/site/camera/tendon/
actuator/sensor spec objects. `nuser_*` fields are direct attributes on
the `MjSpec` object (not under `.size`), defaulting to `-1` (auto-infer).

### EGT-3e: User Data Stress-Test Edge Cases — Round 5

**MuJoCo 3.5.0**, additional edge cases from fifth stress-test round:

**`<frame user="...">` silently discarded:** The `<frame>` element
syntactically accepts `user` but the data never appears in any `*_user`
array. Frame is a compile-time grouping element, not a runtime element.
This is a parser quirk that could trap implementers.

**Explicit user attribute overrides default — even if default is too long:**
When a default has `geom user="1 2 3 4 5"` (5 values) but the geom has
explicit `user="1 2 3"` (3 values) and `nuser_geom=3`, compilation
**succeeds**. The explicit attribute fully overrides the default, so the
default's 5-value length is never checked. Only the effective user data
(after defaults cascade) is validated against nuser_*.

**Two error message formats for too-long validation:**
- Format A (geom, actuator): `"user has more values than nuser_geom in geom 'g1' (id = 0)"` — name+id inline
- Format B (body, joint, site, tendon, sensor): `"user has more values than nuser_body in body"` — name on second line only

**Only first validation error reported:** MuJoCo stops at the first error
in XML document order. If two geoms both have too-long user data, only the
first (by XML position) is reported.

**NaN warning timing:** NaN in user data produces a warning on stderr at
**process exit**, not at parse time. Inf and -inf produce no warning at all.
The model compiles successfully in all cases.

**User data arrays are mutable after compilation:** `m.body_user[1] = [99, 88]`
works at runtime. Arrays are views into model memory (not copies). NaN/Inf
can be written without validation. Changes persist across `mj_step` calls.
However, `nuser_*` fields are read-only after compilation.

**Multiple `<sensor>` blocks merge:** Two separate `<sensor>` blocks in
the same XML are legal — their contents concatenate in document order.
Same behavior as multiple `<size>` elements.

**Mocap bodies accept user data:** `<body mocap="true" user="1 2 3">` is
valid. User data is independent of mocap state.

**Binary save preserves full precision:** `mj_saveModel` / `.mjb` format
preserves bit-exact float64 values. XML save (`mj_saveLastXML`) truncates
to ~6 significant figures.

**Real-world usage:** Only 1 of 56 loadable dm_control/MuJoCo test models
uses user data (a single geom with `user="258"` in a test humanoid).

**Performance:** User data adds <1% overhead to compilation and simulation
with 1000 elements.

### EGT-3f: Conformance Verification — Round 6

**MuJoCo 3.5.0**, conformance verification against rubric claims using
golden models. All 15 tests PASS — no rubric claims contradicted.

**Golden Model 1 (50 checks):** Comprehensive model exercising all 7
element types, auto-sizing + explicit nuser_*, default inheritance,
childclass, explicit class= override, user="", user="0", world body zeros,
freejoint vs `<joint type="free">`, varying user lengths. All 50 checks
PASS with exact numerical match.

**Golden Model 2 (15 checks):** 3-level defaults cascade (root→mid→leaf).
Verified: transitive inheritance through intermediate classes that don't
override, childclass propagation, explicit class= override, explicit user=
override, no-class elements get unnamed root default. All 15 checks PASS.

**Golden Model 3 (9 checks):** nuser_* validation boundaries. Verified:
exact-fit succeeds, too-long-default-inherited fails, too-long-default-
but-all-elements-override succeeds, mixed (one inherits, one overrides)
fails on the inheriting element. All 9 checks PASS.

**Actuator subtype verification (3 checks):** Confirmed shared storage
within a default class (last-write-wins) and separate classes yield
independent values. All 3 checks PASS.

**Multiple `<size>` merge (3 checks):** Confirmed last-writer-wins per
attribute across 2 and 3 `<size>` elements. All 3 checks PASS.

**Auto-sizing algorithm (4 checks):** Confirmed max across inline + defaults
+ overrides. **Critical correction:** auto-sizing is demand-driven — unused
default classes do NOT contribute to nuser_* max. Only resolved user arrays
on instantiated elements count.

**Rubric claim audit (6 tests, all PASS):**
- `user=""` ≡ omitting user (bit-identical in all contexts)
- `user="0"` is `[0.0]` (nuser=1), `user=""` is absence (nuser=0)
- 4-level transitive inheritance A→B→C→D works
- Explicit user overrides too-long default (5 sub-cases all correct)
- XML order of `<size>` vs `<default>` irrelevant (bit-identical output)
- Sensor ordering = XML document order (not type order)

### EGT-4: MuJoCo User Data Element Types

**MuJoCo 3.5.0** (`mjmodel.h` lines 743–750, 809–1222):

8 element types with user data:
- `body_user` (nbody × nuser_body) — line 809
- `jnt_user` (njnt × nuser_jnt) — line 845
- `geom_user` (ngeom × nuser_geom) — line 894
- `site_user` (nsite × nuser_site) — line 906
- `cam_user` (ncam × nuser_cam) — line 925 (**out of scope per umbrella**)
- `tendon_user` (ntendon × nuser_tendon) — line 1167
- `actuator_user` (nu × nuser_actuator) — line 1202
- `sensor_user` (nsensor × nuser_sensor) — line 1222

Flat storage with stride: `element_user[id * nuser_* + idx]`.

### EGT-5: `<size>` Element in MJCF

**dm_control schema.xml lines 105–122**: `<size>` is a direct child of
`<mujoco>` (sibling to `<option>`, `<worldbody>`, etc.), not a child of
`<option>`. Contains `nuser_body`, `nuser_jnt`, `nuser_geom`, `nuser_site`,
`nuser_cam`, `nuser_tendon`, `nuser_actuator`, `nuser_sensor` (all int type).

Also contains `memory`, `njmax`, `nconmax`, `nstack`, `nuserdata`, `nkey` —
all out of scope for this spec.

### EGT-6: `<flexcomp>` Not in MuJoCo 3.5.0

**MuJoCo Python 3.5.0**: `<flexcomp>` produces `Schema violation:
unrecognized element`. Tested with `from_xml_string()` and
`from_xml_path()`. The `MjsFlex` spec object has no `inertiabox`, `scale`,
`quat`, `file`, `pos`, `type`, `count`, `spacing`, `mass`, or `rigid`
attributes.

MuJoCo docs at readthedocs.io DO list `<flexcomp>` as a valid element.
dm_control schema.xml (lines 1163–1242) includes `<flexcomp>` with all
attributes. CortenForge's parser is based on this schema.

**Conclusion:** DT-88 cannot use MuJoCo binary output as ground truth.
Verification is against dm_control schema and MuJoCo documentation only.

### EGT-7: Flexcomp Attribute Definitions (dm_control schema)

From dm_control schema.xml lines 1163–1191:

| Attribute | Schema type | Schema details |
|-----------|-------------|----------------|
| `inertiabox` | `float` | Scalar. Used for vertex inertia: `mass/n * 2*ib²/3` per vertex |
| `scale` | `array float[3]` | `[sx, sy, sz]` non-uniform scale applied to generated vertices |
| `quat` | `array float[4]` | `[w, x, y, z]` rotation quaternion applied to generated vertices |
| `file` | `string` | Path to mesh/grid data file |
| `pos` | `array float[3]` | Position offset (also exists in schema, currently not parsed) |
| `axisangle` | `array float[4]` | Alternative orientation (schema line 1184) |
| `xyaxis` | `array float[6]` | Alternative orientation (schema line 1185) |
| `zaxis` | `array float[3]` | Alternative orientation (schema line 1186) |
| `euler` | `array float[3]` | Alternative orientation (schema line 1187) |
| `origin` | `array float[3]` | Origin for grid generation (schema line 1188) |

DT-88 scope: `inertiabox`, `scale`, `quat`, `file`. The umbrella lists
only these 4. Additional orientation alternatives and `pos`/`origin` are
out of scope (track as future work).

### EGT-8: CortenForge Codebase Context — User Data

| Area | File:Line | Current State |
|------|-----------|---------------|
| Body user parsing | `parser.rs:1607` (`parse_body_attrs`) | **Missing** — no `user` parsing |
| Geom user parsing | `parser.rs:1769` (`parse_geom_attrs`) | **Missing** |
| Joint user parsing | `parser.rs:1663` (`parse_joint_attrs`) | **Missing** |
| Site user parsing | `parser.rs:1933` (`parse_site_attrs`) | **Missing** |
| Tendon user parsing | `parser.rs:3411` (`parse_tendon_attrs`) | **Missing** |
| Actuator user parsing | `parser.rs:2121` (`parse_actuator_attrs`) | **Missing** |
| Sensor user parsing | `parser.rs:3630` (`parse_sensor_attrs`) | **Already implemented** (`sensor.user = parts`) |
| Sensor defaults user | `parser.rs:962` (`parse_sensor_defaults`) | **Already implemented** |
| `<size>` parsing | `parser.rs:90–157` (main parse loop) | **Missing** — `<size>` falls to `_ => skip_element` on line 132 |
| MjcfBody struct | `types.rs:2217` | **No user field** |
| MjcfGeom struct | `types.rs:1170` | **No user field** |
| MjcfJoint struct | `types.rs:1407` | **No user field** |
| MjcfSite struct | `types.rs:1530` | **No user field** |
| MjcfActuator struct | `types.rs:2400` | **No user field** |
| MjcfTendon struct | `types.rs:2750` | **No user field** |
| MjcfSensor struct | `types.rs:3178` | **Has `user: Vec<f64>`** |
| MjcfSensorDefaults | `types.rs:815` | **Has `user: Option<Vec<f64>>`** |
| Other defaults structs | `types.rs:584–979` | **No user fields** on JointDefaults (584), GeomDefaults (626), ActuatorDefaults (686), TendonDefaults (771), SiteDefaults (951). MuJoCo schema supports `user` in defaults for: joint, geom, site, tendon, actuator. NOT for body or sensor. |
| MjcfOption/MjcfModel | `types.rs:365+`, `types.rs:3807` | **No `nuser_*` fields; no `<size>` data** |
| Model (sim-core) | `model.rs:44` | **No `*_user` or `nuser_*` fields** |
| model_init.rs | entire file | **No user data initialization** |
| Builder pipeline | `builder/` directory | **No `.user` references** — sensor user parsed but never wired to Model |

### EGT-9: CortenForge Codebase Context — Flexcomp

| Area | File:Line | Current State |
|------|-----------|---------------|
| Flexcomp parsing | `parser.rs:2902` (`parse_flexcomp`) | **Exists** — parses `type`, `count`, `spacing` via child calls |
| `parse_flex_attrs` | `parser.rs:2772` | Parses: name, dim, radius, body, node, group, mass. **Missing:** inertiabox, scale, quat, file, pos |
| `parse_flexcomp_empty` | `parser.rs:2986` | Same as above — parses attrs + generates mesh |
| `MjcfFlex` struct | `types.rs:3669` | **No fields** for inertiabox, scale, quat, file |
| Grid generation | `parser.rs:3021` (`generate_grid`) | Generates vertices from count×spacing. **No scale/quat/pos transform applied** |
| Box generation | `parser.rs:3050` (`generate_box_mesh`) | Same — no transforms |
| Flex builder | `builder/flex.rs:23` (`process_flex_bodies`) | Processes MjcfFlex → Model. No flexcomp-specific logic |

### EGT-10: Existing Defaults Struct Pattern

All 7 defaults structs follow the same pattern (`types.rs`):

```rust
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MjcfXxxDefaults {
    pub field: Option<T>,  // Option for defaultable attributes
    // ...
}
```

Defaults application follows the `apply_defaults()` pattern in
`builder/defaults.rs`. Each element type has an `apply_to_xxx()` function
that applies defaults class values when the element's field is None.

For user data defaults, the pattern would be:
- `user: Option<Vec<f64>>` on each defaults struct
- `apply_to_xxx()` copies default user data when element's user is None
- Element's user data overrides entirely (no merging)

---

## Criteria

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does — exact field names, sizes,
> auto-sizing logic, padding behavior, default inheritance, validation
> rules, and storage layout. For §55 (user data), this is the primary
> quality gate. For DT-88 (flexcomp), reference fidelity is against
> documentation and dm_control schema only.

| Grade | Bar |
|-------|-----|
| **A+** | **§55:** Every user data field cited with `mjmodel.h` line number. Auto-sizing logic (EGT-1) described with concrete examples showing the max-length computation — demand-driven: sizes from resolved user arrays on instantiated elements, NOT from all declared defaults (EGT-3d corrected in EGT-3f). Zero-padding behavior explicit for: elements with shorter user data, elements with no user data, world body. Error-on-too-long behavior (EGT-2) described with exact error condition (including error when default inherits too-long data — EGT-3a; explicit user overrides too-long default — EGT-3e). Default class inheritance (EGT-3) described with: full-replacement semantics (not partial merge), which types support defaults (5 of 7), body/sensor exceptions, `user=""` does NOT clear defaults (EGT-3a), transitive inheritance through intermediate classes (EGT-3d), `childclass` does NOT shadow root default (EGT-3d), `class="main"` references root default (EGT-3d). Actuator subtype defaults share storage (last-write-wins within a default class — EGT-3b). `<size>` element parsing for `nuser_*` attributes described including: `nuser_*=-1` sentinel (only valid negative — EGT-3b), strict integer parsing (floats rejected — EGT-3b), max value 2147483646 (EGT-3d), multiple `<size>` elements merge (EGT-3b). `childclass` propagation with nearest-ancestor rule (EGT-3b/3c). `<freejoint>` does NOT accept `user` — must use `<joint type="free">` (EGT-3c). `user="0"` is data (`[0.0]`), `user=""` is absence of data (EGT-3c). Hex notation accepted (`0xFF` — EGT-3d). All 7 in-scope element types enumerated; exhaustive verification no others accept `user` (EGT-3d). Element ordering is depth-first pre-order (EGT-3c). All claims verified by golden model conformance tests (EGT-3f). **DT-88:** All 4 attributes cited with dm_control schema line numbers. Transform application order (scale then rotate) documented. Default values stated. Split mandate boundary explicit. `<flex>` element (not `<flexcomp>`) confirmed present in 3.5.0 (EGT-3b). |
| **A** | Correct behavior described for all features. Minor gaps in edge-case coverage. |
| **B** | High-level correct but missing auto-sizing algorithm or padding details. |
| **C** | Assumes flat storage or gets element types wrong. |

### P2. Algorithm Completeness

> Every algorithmic step is specified unambiguously. For §55: the auto-sizing
> computation, the pad/truncate logic, the validation check, the defaults
> cascade, and the builder wiring pipeline. For DT-88: the vertex
> transform pipeline (scale → quat rotate → pos offset).

| Grade | Bar |
|-------|-----|
| **A+** | Auto-sizing algorithm written as pseudocode: `nuser = max(user_lengths) if no explicit nuser, else explicit_nuser; validate all lengths ≤ nuser; pad shorter arrays with 0.0`. Builder wiring pseudocode: `for each element { model.*_user.push(padded_user) }`. Defaults cascade: `if element.user.is_none() { use default.user } else { use element.user }`. Flexcomp transform: `v' = quat.rotate(scale ⊙ v) + pos`. No "see MuJoCo source" gaps. |
| **A** | Complete algorithms with one or two implicit details. |
| **B** | Algorithm structure clear but auto-sizing or defaults cascade hand-waved. |
| **C** | Skeleton only. |

### P3. Convention Awareness

> Spec addresses MuJoCo → CortenForge translations for: field naming
> (MuJoCo `jnt_user` → CortenForge `jnt_user`), storage layout (flat →
> `Vec<Vec<f64>>`), quaternion convention (w,x,y,z), and `nuser_*` int
> type (MuJoCo `mjtSize` → Rust `i32`).

| Grade | Bar |
|-------|-----|
| **A+** | Convention table present mapping: MuJoCo field name → CortenForge field name for all 7 element types and `nuser_*` fields. Storage layout difference documented (MuJoCo flat `mjtNum*` with stride vs CortenForge `Vec<Vec<f64>>`). Quaternion convention for flexcomp quat stated (MuJoCo w,x,y,z). Index convention for user arrays (body_user[body_id]) explicit. `nuser_*` default value difference documented (MuJoCo MJCF default -1, compiled default ≥ 0; CortenForge stores compiled value). |
| **A** | Major conventions documented. Minor field-name mappings left to implementer. |
| **B** | Some conventions noted, storage layout difference not addressed. |
| **C** | MuJoCo names used without adaptation. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable. §55 ACs have concrete
> values verified against MuJoCo 3.5.0 output. DT-88 ACs verify structural
> parsing against schema definitions.

| Grade | Bar |
|-------|-----|
| **A+** | Every §55 runtime AC has three-part structure: (1) specific MJCF input model, (2) expected `nuser_*` and `*_user` array values verified against MuJoCo 3.5.0 output, (3) which Model field to check. At least one AC for: auto-sizing (no explicit nuser_*), explicit nuser_* with padding, too-long error, default inheritance (including full-replacement semantics), auto-sizing includes defaults lengths, nuser_* smaller than default causes error, world body zeros, no-user-data-at-all (nuser=0), `nuser_*=-1` triggers auto-sizing, `user=""` does not clear defaults (inherits default values — EGT-3a), actuator subtype defaults last-write-wins (EGT-3b), `childclass` propagation of user data with nearest-ancestor rule (EGT-3b/3c), `nuser_*` strict integer validation (floats/negative < -1 rejected — EGT-3b), multiple `<size>` elements merge (EGT-3b), `<freejoint>` rejects `user` vs `<joint type="free">` accepts it (EGT-3c), `user="0"` is `[0.0]` not absence of data (EGT-3c), transitive inheritance through intermediate classes (EGT-3d), `childclass` does not shadow root default (EGT-3d), hex notation in user values (EGT-3d). DT-88 ACs verify parsed struct fields against schema-defined types and defaults. Code-review ACs labeled as such. |
| **A** | ACs are testable. Some lack exact numerical expectations. |
| **B** | ACs are directionally correct but vague. |
| **C** | ACs are aspirational. |

### P5. Test Plan Coverage

> Tests cover happy path, edge cases, default inheritance, error cases,
> and flexcomp transforms. Each AC maps to at least one test.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present. Explicit edge case inventory: world body user data (always zeros), no-user-data model (nuser=0), too-long user array error, single-value user, default inheritance with shorter override, default inheritance with no override, multiple elements with different user lengths, `<size nuser_*="0">` explicit zero, `nuser_*=-1` auto-sizing, `user=""` with defaults (inherits default, not empty — EGT-3a), nuser_* smaller than default user length (error on inheritance — EGT-3a), auto-sizing includes defaults lengths, override is full replacement not partial merge, nested default classes, `childclass` propagation to descendants with nearest-ancestor rule (EGT-3b/3c), actuator subtype defaults last-write-wins (EGT-3b), `nuser_*` with float value rejected (EGT-3b), `nuser_*=-2` rejected (EGT-3b), multiple `<size>` elements merge (EGT-3b), whitespace variants in user data (tabs/newlines — EGT-3b), `<freejoint>` rejects `user` attribute (EGT-3c), `user="0"` is `[0.0]` not empty (EGT-3c), NaN/Inf accepted with warning (EGT-3c), commas/semicolons rejected as separators (EGT-3c), user data physics inertness empirically verified (EGT-3c), depth-first element ordering in `*_user` arrays (EGT-3c), transitive default inheritance through intermediate classes (EGT-3d), `childclass` does not shadow root default (EGT-3d), `class="main"` references root default (EGT-3d), hex notation `0xFF` in user data (EGT-3d), auto-sizing from defaults only (EGT-3d), denormal float rejection (EGT-3d). At least one MuJoCo conformance test per element type (same model, verify output matches MuJoCo). Negative case: user data has no runtime physics effect (bit-identical qpos/qvel — EGT-3c). Flexcomp: scale/quat transform verified on generated vertices. Supplementary tests justified. |
| **A** | Good coverage. Minor edge-case gaps. |
| **B** | Happy path covered. Edge cases sparse. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering constraints, and interactions with other specs
> are explicitly stated. §55 and DT-88 are independent within Spec C.

| Grade | Bar |
|-------|-----|
| **A+** | Execution order unambiguous. `<size>` parsing (S1) must precede element user parsing (S2+) because nuser_* affects validation. Flexcomp (DT-88) independent of user data (§55). Cross-spec note: no interaction with Spec A defaults or Spec B joint physics. Prerequisites include commit hashes for Phase 7 work already landed. |
| **A** | Order is clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Spec identifies every file touched, every behavior change, and every
> existing test that might break.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list with per-file change description. Behavioral changes: `<size>` parsing now active (previously skipped), 6 element types gain `user` field (body/geom/joint/site/tendon/actuator), 5 defaults structs gain `user: Option<Vec<f64>>` (joint/geom/site/tendon/actuator — body has no defaults, sensor already has it), MjcfFlex gains 4 fields, Model gains 14+ new fields. Existing test impact: flexcomp tests may need updating if scale/quat changes vertex positions, sensor user data tests should be unaffected. Domain test baseline (2,100+ tests) impact assessed. `model_init.rs` additions enumerated. |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions within the spec. Field names, counts, and cross-references
> are consistent throughout.

| Grade | Bar |
|-------|-----|
| **A+** | All 7 element types listed identically in MuJoCo Reference, Specification, Files Affected, and Test Plan. `nuser_*` field names consistent (not mixing `nuser_joint` with `nuser_jnt`). AC numbers match between AC section and Traceability Matrix. Edge case lists consistent across sections. Flexcomp attribute types consistent (inertiabox as f64 everywhere, not f64 in one place and Vec3 in another). |
| **A** | Consistent. One or two minor inconsistencies. |
| **B** | Some sections use different names for the same concept. |
| **C** | Contradictions between sections. |

### P9. Data Lifecycle Completeness

> New fields must be parsed, stored on MJCF types, applied through defaults,
> wired through the builder to Model, and initialized in model_init.rs.
> Missing any stage is a silent bug (field exists but is always default).

| Grade | Bar |
|-------|-----|
| **A+** | Every new field has explicit lifecycle documentation: (1) parsed in parser.rs, (2) stored on MJCF type struct, (3) defaults application in defaults.rs, (4) builder wiring to Model field, (5) Model field initialization in model_init.rs. For each of the 7 user data element types, all 5 stages are specified. For flexcomp 4 attributes, stages 1–2 specified (3–5 may be N/A depending on whether flexcomp attrs transfer to Model). Lifecycle table present. |
| **A** | All stages covered. One or two stages left implicit. |
| **B** | Parse and store covered. Builder wiring or defaults cascade missing. |
| **C** | Only parsing addressed. |

### P10. Element Type Completeness

> §55 touches 7 element types that follow nearly identical patterns. The
> spec must be exhaustive — missing one element type is a conformance gap.
> Each type must be explicitly enumerated with its specific parser function,
> MJCF struct, defaults struct, and Model field.

| Grade | Bar |
|-------|-----|
| **A+** | Table enumerating all 7 element types with columns: element type, MuJoCo C field name, CortenForge MJCF struct, parser function, defaults struct (or "N/A" for body), defaults parse function, Model field, nuser field. Every cell filled. Sensor explicitly called out as parse-exists/wire-missing. Body explicitly noted as no-defaults (EGT-3). No type silently omitted. Camera explicitly stated as out of scope with justification. |
| **A** | All 7 types enumerated. Minor details left implicit. |
| **B** | Most types covered. One or two missing or hand-waved. |
| **C** | Only a subset of element types addressed. |

---

## Rubric Self-Audit

### Self-audit checklist

- [x] **Specificity:** Every A+ bar names specific MuJoCo fields
      (`body_user`, `nuser_body`), specific parser functions
      (`parse_body_attrs`, `parse_geom_attrs`), specific EGT entries
      (EGT-1 auto-sizing, EGT-2 error, EGT-3 defaults). Two reviewers
      would agree on grades by pointing to specific spec content.

- [x] **Non-overlap:** P1 grades MuJoCo reference accuracy; P2 grades
      algorithm implementation completeness; P9 grades data lifecycle
      (parse→store→default→wire→init stages); P10 grades element type
      completeness (all 7 types). Boundary: P1 asks "is the MuJoCo
      behavior described correctly?"; P9 asks "does the spec cover all 5
      pipeline stages for each field?"; P10 asks "are all 7 element types
      enumerated?".

- [x] **Completeness:** P1–P8 standard criteria plus P9 (data lifecycle)
      and P10 (element type completeness). The two domain-specific criteria
      cover the task's unique dimensions: many pipeline stages per field
      (P9) and many nearly-identical element types (P10). A spec could be
      A+ on P1–P8 but miss a lifecycle stage or element type — P9/P10
      catch those gaps.

- [x] **Gradeability:** P1 → MuJoCo Reference + Key Behaviors; P2 →
      Specification sections; P3 → Convention Notes; P4 → Acceptance
      Criteria; P5 → Test Plan + Traceability; P6 → Prerequisites +
      Execution Order; P7 → Risk & Blast Radius; P8 → cross-cutting;
      P9 → Lifecycle table in Specification; P10 → Element Type table.

- [x] **Conformance primacy:** P1 tailored with `mjmodel.h` field names,
      line numbers, and specific MuJoCo behaviors (auto-sizing, padding,
      error-on-too-long, default inheritance). P4 requires MuJoCo-verified
      expected values for §55 ACs. P5 requires at least one MuJoCo
      conformance test per element type. DT-88 split mandate documented.

- [x] **Empirical grounding:** EGT-1 through EGT-10 plus EGT-3a/3b/3c/3d/3e/3f
      filled in with MuJoCo 3.5.0 empirical results (138 stress tests across
      5 exploration rounds + 1 conformance verification round) and codebase
      context. Every A+ bar referencing MuJoCo behavior has a corresponding
      EGT entry. DT-88 absence from MuJoCo 3.5.0 binary verified empirically
      (EGT-6). Rounds 2–6 uncovered 26 new gaps (R8–R33) now resolved.
      Golden model conformance tests (EGT-3f) verified all rubric claims —
      0 contradictions, 1 correction (demand-driven auto-sizing — R32).

### Criterion → Spec section mapping

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P1 | MuJoCo Reference, Key Behaviors table, Convention Notes |
| P2 | Specification (S1, S2, ...) |
| P3 | Convention Notes, Specification code |
| P4 | Acceptance Criteria |
| P5 | Test Plan, AC→Test Traceability Matrix, Edge Case Inventory |
| P6 | Prerequisites, Execution Order |
| P7 | Risk & Blast Radius (Behavioral Changes, Files Affected, Existing Test Impact) |
| P8 | *Cross-cutting — all sections checked for mutual consistency* |
| P9 | Lifecycle table in Specification, model_init.rs entries |
| P10 | Element Type table, Files Affected per-type breakdown |

---

## Scorecard

| Criterion | Grade | Evidence |
|-----------|-------|----------|
| P1. MuJoCo Reference Fidelity | | |
| P2. Algorithm Completeness | | |
| P3. Convention Awareness | | |
| P4. Acceptance Criteria Rigor | | |
| P5. Test Plan Coverage | | |
| P6. Dependency Clarity | | |
| P7. Blast Radius & Risk | | |
| P8. Internal Consistency | | |
| P9. Data Lifecycle Completeness | | |
| P10. Element Type Completeness | | |

**Overall: — (Rev 6)**

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| R1 | Scope | DT-88 `<flexcomp>` not in MuJoCo 3.5.0 binary | EGT verification (Python `from_xml_string` schema violation) | Split mandate: §55 full conformance, DT-88 documentation fidelity only | Rubric Rev 1 |
| R2 | P1 | `inertiabox` type discrepancy: dm_control schema says `float`, MuJoCo docs say `real(3)` | EGT verification (schema.xml line 1173 vs readthedocs) | Use dm_control schema (`float` = scalar) — CortenForge parser targets dm_control schema | Rubric Rev 1 |
| R3 | P10 | Camera `cam_user`/`nuser_cam` exists in MuJoCo but umbrella excludes it | EGT verification (mjmodel.h line 925) | Explicitly out of scope per umbrella; track as future work | Rubric Rev 1 |
| R4 | Scope | Flexcomp `pos`, `axisangle`, `xyaxis`, `zaxis`, `euler`, `origin` also missing from parser | EGT-7 schema review | Out of scope per umbrella (only 4 attributes listed). Track as future work. | Rubric Rev 1 |
| R5 | EGT | Sensor user data already parsed but NOT wired to Model | Codebase search (EGT-8) | Spec must document sensor as "parse exists, wire missing" — not "already complete" | Rubric Rev 1 |
| R6 | P10/EGT-3 | Body and sensor do NOT support user defaults in MuJoCo XML | Empirical verification (`<default><body>` and `<default><sensor>` both produce schema violation) | P10 requires defaults column in element type table; body=N/A, sensor=existing. Only 5 of 7 types need new defaults struct user fields. | Rubric Rev 1 |
| R7 | P9 | `apply_to_body()` does not exist in defaults.rs — bodies are not defaultable | Codebase search (no `apply_to_body` in defaults.rs) | Data lifecycle for body user data skips stage 3 (defaults). Spec must note this difference. | Rubric Rev 1 |
| R8 | P1 | `user=""` does NOT clear defaults — treated as "not specified", defaults still apply | Round 2 stress test (default user="1 2 3" + element user="" → gets [1,2,3]) | EGT-3a updated. P1/P4/P5 A+ bars require spec to document this. Parser must treat `user=""` as None. | Rubric Rev 2 |
| R9 | P1 | Actuator subtype defaults share storage — `<motor>`, `<position>`, `<general>` etc. are aliases within a default class; last-write-wins | Round 2 stress test (all subtypes share same default element) | EGT-3b added. Spec must document shared-storage semantics. Implementation: all actuator subtypes map to same defaults struct. | Rubric Rev 2 |
| R10 | P1 | `nuser_*` strict integer parsing — floats and negatives < -1 rejected | Round 2 stress test ("1.0" → error, "-2" → error) | EGT-3b added. Spec must require strict integer parsing for `<size>` nuser_* attributes. | Rubric Rev 2 |
| R11 | P1 | Multiple `<size>` elements merge with last-writer-wins per attribute | Round 2 stress test (two `<size>` elements with overlapping attributes) | EGT-3b added. Parser must handle multiple `<size>` elements. | Rubric Rev 2 |
| R12 | EGT | `<flex>` element (not `<flexcomp>`) confirmed present in MuJoCo 3.5.0 | Round 2 stress test (`<flex>` with body/vertex/element attrs accepted) | EGT-3b added. DT-88 scope unchanged (flexcomp attrs only) but context important. | Rubric Rev 2 |
| R13 | P5 | `childclass` propagation of user data not in original edge case inventory | Round 2 stress test (childclass="foo" propagates to all descendant elements) | EGT-3b added. P5 A+ bar updated to require childclass test case. | Rubric Rev 2 |
| R14 | P1 | `<freejoint>` does NOT accept `user` attribute — reduced attribute set | Round 3 stress test (`<freejoint user="...">` → schema violation) | EGT-3c added. Spec must document: use `<joint type="free" user="..."/>` for free joint user data. | Rubric Rev 3 |
| R15 | P1/P5 | NaN and Inf accepted in user data (with warning, case-insensitive) | Round 3 stress test (`user="nan inf -inf"` accepted) | EGT-3c added. Parser should accept NaN/Inf. Add to edge case inventory. | Rubric Rev 3 |
| R16 | P5 | Only whitespace separators valid in user data — commas/semicolons rejected | Round 3 stress test (`user="1,2,3"` → "bad format") | EGT-3c added. Parser must use whitespace-only splitting. | Rubric Rev 3 |
| R17 | Scope | `<key>` does NOT accept `user` — no per-element user data in keyframes | Round 3 stress test (`<key user="...">` → schema violation) | EGT-3c added. Out of scope (keyframes not part of §55). | Rubric Rev 3 |
| R18 | P1 | `user="0"` is `[0.0]` (1-element array), NOT absence of data | Round 3 stress test (`user="0"` → nuser=1, `user=""` → nuser=0) | EGT-3c added. Parser must distinguish `user="0"` from `user=""`. Critical semantic distinction. | Rubric Rev 3 |
| R19 | P1 | Body/element ordering in `*_user` arrays is depth-first pre-order | Round 3 stress test (10-level body hierarchy, sensor XML order) | EGT-3c added. Spec must document ordering for builder wiring. | Rubric Rev 3 |
| R20 | P1/EGT-3 | `childclass` follows nearest-ancestor rule, not root | Round 3 stress test (nested childclass="A" → childclass="B") | EGT-3c added. Refines EGT-3b childclass documentation. | Rubric Rev 3 |
| R21 | P1 | Transitive default inheritance — chain propagates through intermediate classes that don't override user | Round 4 stress test (A→B→C, B has no user, C gets A's user) | EGT-3d added. Spec must document transitive inheritance. | Rubric Rev 4 |
| R22 | P1/EGT-3 | `childclass` does NOT shadow root default — class inherits from ancestors | Round 4 stress test (childclass="myclass" with no user, root has user → element gets root's user) | EGT-3d added. Spec must document that childclass selects a class, not blocks inheritance. | Rubric Rev 4 |
| R23 | P1 | `class="main"` explicitly references root default class | Round 4 stress test (class="main" overrides childclass, gets root default values) | EGT-3d added. Spec must document class="main" as escape from childclass. | Rubric Rev 4 |
| R24 | P1 | `nuser_*` max value is 2147483646 (strict `< INT_MAX`) | Round 4 stress test (2147483647 rejected, 2147483646 accepted) | EGT-3d added. Spec should document i32 range. | Rubric Rev 4 |
| R25 | P1/P5 | Hex notation (`0xFF`, `0x1p10`) accepted in user data via strtod | Round 4 stress test (0xFF → 255.0, 0x1p10 → 1024.0) | EGT-3d added. Parser note: Rust f64 parsing does NOT support hex by default — may need special handling or documenting as known deviation. | Rubric Rev 4 |
| R26 | P5 | Denormal/subnormal float values rejected by MuJoCo parser | Round 4 stress test (1e-308, 5e-324 rejected; DBL_MIN with full precision accepted) | EGT-3d added. Edge case for test plan. | Rubric Rev 4 |
| R27 | P10 | Exhaustive verification: NO element types beyond the 8 accept `user` | Round 4 stress test (tested inertial, light, flex, material, texture, mesh, hfield, equality elements — all rejected) | EGT-3d added. Confirms exhaustive 8-type list. Strengthens P10 completeness. | Rubric Rev 4 |
| R28 | P1 | `<frame user="...">` silently discarded — parser accepts but data never appears in any `*_user` array | Round 5 stress test (frame is compile-time grouping, not runtime element) | EGT-3e added. Spec should note as known parser quirk to avoid. | Rubric Rev 5 |
| R29 | P1 | Explicit user attribute overrides default even if default is too long — only effective user length is validated | Round 5 stress test (default user=5 vals, element user=3 vals, nuser=3 → succeeds) | EGT-3e added. Spec must document: validation checks effective (post-override) length, not default length. | Rubric Rev 5 |
| R30 | P4 | Two distinct error message formats for too-long validation (Format A: geom/actuator with name inline; Format B: other types) | Round 5 stress test (all 7 types tested) | EGT-3e added. Conformance tests should match exact error format per type. | Rubric Rev 5 |
| R31 | P4 | Only first validation error reported — MuJoCo stops at first error in XML document order | Round 5 stress test (two too-long geoms, only first reported) | EGT-3e added. Context for test expectations. | Rubric Rev 5 |
| R32 | P1/P2 | Auto-sizing is demand-driven: unused default classes do NOT contribute to nuser_* max | Round 6 conformance (named class with user="1..10", no geom references it → nuser_geom ignores it) | EGT-3d corrected, EGT-3f added. **Critical**: spec must describe auto-sizing as operating on resolved/effective user arrays of instantiated elements, not on all declared defaults. | Rubric Rev 6 |
| R33 | P1 | Validation is per-element not per-default: too-long default stored without error, validated only when inherited | Round 6 conformance (default user=5, nuser=3, all geoms have explicit user ≤ 3 → succeeds) | EGT-3f added. Reinforces R29. | Rubric Rev 6 |
