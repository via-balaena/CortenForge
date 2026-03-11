# Composite Body Generation (§46) — Spec

**Status:** Approved
**Phase:** Roadmap Phase 13 — Remaining Core
**Effort:** M (reduced from XL — only cable type remains non-deprecated)
**MuJoCo ref:** `mjCComposite::Make()` in `user_composite.cc`, lines 131–239;
`MakeCable()` lines 243–313; `AddCableBody()` lines 317–445;
`mjuu_updateFrame()` in `user_util.cc`
**MuJoCo version:** 3.4.0
**Test baseline:** 2,273+ sim domain tests
**Prerequisites:**
- Phase 13 Specs A+B (landed — conformance gate green for PGS)
- Frame expansion infrastructure (`builder/frame.rs` — already landed)
- Contact exclude support (`MjcfContactExclude` — already landed)
- None pending

**Independence:** This spec is independent of Spec D (plugin system). Shared
files: `parser.rs` and `builder/mod.rs` are modified by both, but at different
code locations with no overlap. Spec C adds composite parsing/expansion; Spec D
adds plugin parsing/dispatch.

> **Conformance mandate:** This spec exists to make CortenForge produce
> numerically identical model structure to MuJoCo for `<composite type="cable">`
> elements. The MuJoCo C source code is the single source of truth.

---

## Scope Adjustment

| Umbrella claim | MuJoCo 3.4.0 reality | Action |
|----------------|---------------------|--------|
| 7 composite types (grid, rope, cable, cloth, box, cylinder, ellipsoid) | Only `cable` is non-deprecated. 5 types return error. box/cylinder/ellipsoid are `flexcomp` types, never `composite` types. | In scope: `cable` + deprecation errors |
| Tendon generation | Only deprecated `cloth` generated tendons. Cable does not. | Drop |
| Skin generation | Box-geom cables only. Complex bicubic interpolation. Rendering-only. | Defer — DT-165 |
| Plugin support (`mujoco.elasticity.cable`) | Requires plugin system (Spec D). | Defer to post-Spec-D |
| `<flexcomp>` element | Separate infrastructure from `<composite>`. Not in §46. | Defer — separate spec |
| `<replicate>` element | Separate element replacing `particle` composite. | Defer — separate spec |

**Final scope:**
1. Parse `<composite>` element with all attributes and child elements
2. Cable body chain generation (bodies, geoms, joints, sites, excludes, custom text)
3. Curve-based vertex generation (4 shapes: LINE, COS, SIN, ZERO)
4. Discrete Bishop frame propagation (rotation-minimizing frame along polyline)
5. Deprecation errors for non-cable types matching MuJoCo's exact messages
6. Pipeline integration after frame expansion

---

## Problem Statement

**Conformance gap** — MuJoCo implements `<composite type="cable">` in
`user_composite.cc`; CortenForge does not implement it. The parser silently
skips `<composite>` elements via `skip_element()`. Any MJCF model using
`<composite>` loads without error but is missing its cable bodies, joints,
geoms, sites, and contact exclusions — producing a structurally different
model than MuJoCo.

This is a format completeness gap. Cable composites are used in deformable
body simulations (soft robots, tethers, whips). Without this feature,
models that use `<composite type="cable">` cannot be loaded.

---

## MuJoCo Reference

### Overview

MuJoCo's `<composite>` element is a macro-expansion directive. During model
compilation, the `mjCComposite::Make()` function generates explicit model
elements (bodies, joints, geoms, sites, excludes) from the composite
specification. After expansion, the composite element itself is gone — the
compiled model contains only the generated elements.

In MuJoCo 3.4.0, only `type="cable"` dispatches to actual generation code.
All other types (`particle`, `grid`, `rope`, `loop`, `cloth`) return
deprecation errors directing users to modern replacements.

### `mjCComposite::Make()` — Dispatch and validation

**Source:** `user_composite.cc:131–239`

Validates inputs before dispatching:
1. **Count validation:** All `count[i] >= 1` (line 135–138)
2. **Size validation:** `dot(size, size) >= mjMINVAL` unless `uservert` is
   provided (line 142–144)
3. **Uservert/count exclusion:** If `uservert` is non-empty, `count[0]` must
   be 1 (then overwritten to `uservert.size()/3`). Cannot specify both (line
   147–153)
4. **Dimensionality:** Counts non-1 dimensions. Singleton counts must come
   last (e.g., `[5,1,1]` is valid, `[1,5,1]` is invalid) (line 156–166)
5. **Subgrid constraint:** If skin with subgrid, need 3x3 minimum (line 178)
6. **Plugin check:** Only cable supports plugins, only
   `mujoco.elasticity.cable` (line 186–193)
7. **Type dispatch (line 202–238):**

```c
switch (type) {
    case mjCOMPTYPE_PARTICLE: return comperr("...deprecated...replicate...");
    case mjCOMPTYPE_GRID:     return comperr("...deprecated...flex...");
    case mjCOMPTYPE_ROPE:     return comperr("...deprecated...cable...");
    case mjCOMPTYPE_LOOP:     return comperr("...deprecated...flexcomp...");
    case mjCOMPTYPE_CABLE:    return MakeCable(model, body, error, error_sz);
    case mjCOMPTYPE_CLOTH:    return comperr("...deprecated...shell...");
    default: return comperr("Unknown shape in composite");
}
```

### `MakeCable()` — Cable generation entry

**Source:** `user_composite.cc:243–313`

1. **Validate dim == 1** (cable is strictly 1D) (line 245)
2. **Validate geom type** is cylinder, capsule, or box (line 250–253)
3. **Add custom text** `composite_{prefix}` with data `rope_{prefix}` (line
   257–259)
4. **Generate vertices** if `uservert` is empty (line 262–288):

```c
for (int ix = 0; ix < count[0]; ix++) {
    double v[3];
    for (int k = 0; k < 3; k++) {
        switch (curve[k]) {
            case LINE: v[k] = ix * size[0] / (count[0]-1); break;
            case COS:  v[k] = size[1] * cos(PI * ix * size[2] / (count[0]-1)); break;
            case SIN:  v[k] = size[1] * sin(PI * ix * size[2] / (count[0]-1)); break;
            case ZERO: v[k] = 0; break;
        }
    }
    mjuu_rotVecQuat(v, v, quat);  // rotate by composite orientation
    uservert.push_back(v[0..3]);
}
```

5. **Create bodies:** Loop `ix = 0..count[0]-2`, calling `AddCableBody()`
   for each segment. Body pointer chains: each call returns the new body,
   which becomes the parent for the next call (line 296–298).

### `AddCableBody()` — Per-segment body creation

**Source:** `user_composite.cc:317–445`

Creates one body with its geom, joint, exclude, and optional site.

**Naming convention:**

| Position | Body | Joint | Geom | Site |
|----------|------|-------|------|------|
| First (ix=0) | `{prefix}B_first` | `{prefix}J_first` | `{prefix}G0` | `{prefix}S_first` |
| Middle (ix=1..N-3) | `{prefix}B_{ix}` | `{prefix}J_{ix}` | `{prefix}G{ix}` | — |
| Second-to-last (ix=N-3) | `{prefix}B_{ix}` | `{prefix}J_{ix}` | `{prefix}G{ix}` | — |
| Last (ix=N-2) | `{prefix}B_last` | `{prefix}J_last` | `{prefix}G{ix}` | `{prefix}S_last` |

The "next body" name (used for excludes) follows the same pattern shifted by 1.

**Edge vector computation (line 330–345):**
```c
edge = uservert[3*(ix+1)] - uservert[3*ix];   // edge from ix to ix+1
tprev = uservert[3*ix] - uservert[3*(ix-1)];  // previous tangent (if !first)
tnext = uservert[3*(ix+2)] - uservert[3*(ix+1)]; // next tangent (if !last)
length_prev = normalize(tprev);
normalize(tnext);
```

**Frame update (line 348):**
```c
double length = mjuu_updateFrame(this_quat, normal, edge, tprev, tnext, first);
```

**Body positioning (line 373–389):**
```c
if (first) {
    body->pos = offset + uservert[3*ix .. 3*ix+3];
    body->quat = this_quat;
} else {
    body->pos = [length_prev, 0, 0];  // along parent's local x-axis
    dquat = conjugate(prev_quat) * this_quat;
    body->quat = dquat;
}
```

**Geom placement (line 392–403):**
- Capsule/cylinder: `fromto = [0,0,0, length,0,0]` (along local x-axis)
- Box: `pos = [length/2, 0, 0]`, `size[0] = length/2`

**Joint (line 417–426):**
```c
if (!first || initial != "none") {
    jnt->type = (first && initial == "free") ? FREE : BALL;
    if (jnt->type == FREE) { jnt->damping = 0; jnt->armature = 0; jnt->frictionloss = 0; }
}
```

**Exclude (line 429–433):** Pair `(this_body, next_body)` for all except last
body. Total: `count[0]-2` exclude pairs (= num_cable_bodies - 1).

**Sites (line 436–442):** On first and last bodies only.
First: `pos = [0, 0, 0]`. Last: `pos = [length, 0, 0]`.

### `mjuu_updateFrame()` — Discrete Bishop frame

**Source:** `user_util.cc`

Computes a rotation-minimizing frame along a polyline:

```c
// Normalize tangent
tangent = normalize(edge);

if (first) {
    binormal = normalize(cross(tangent, tnext));
    normal = normalize(cross(binormal, tangent));
} else {
    // Darboux rotation: rotate normal about vertex binormal
    binormal_axis = cross(tprev, tangent);
    angle = atan2(normalize(binormal_axis), dot(tprev, tangent));
    normal = rotate_by_axis_angle(normal, binormal_axis, angle);
    normal = normalize(normal);
    binormal = normalize(cross(tangent, normal));
}
quat = frame_to_quat(tangent, normal, binormal);
return length(edge);
```

**Edge case — straight cable:** When consecutive edges are parallel,
`cross(tangent, tnext) = 0` on first segment. The normal and binormal
degenerate to zero. MuJoCo does not special-case this — the resulting
quaternion may contain NaN components. In practice, straight cables work
because body positions are correct (along local x) regardless of frame
orientation, and ball joints allow any relative rotation.

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| `<composite type="cable">` parsing | Parsed in `mjXReader`, generates bodies via `mjCComposite::Make()` | Silently skipped via `skip_element()` |
| Deprecated composite types | Return specific error messages per type | Silently skipped |
| Cable body chain | N-1 bodies in linear parent chain | Not implemented |
| Bishop frame propagation | `mjuu_updateFrame()` computes rotation-minimizing frame | Not implemented |
| Cable vertex generation | 4 curve shapes (LINE/COS/SIN/ZERO) with quat rotation | Not implemented |
| Contact excludes for adjacent bodies | N-2 exclude pairs auto-generated | Not implemented |
| Custom text metadata | `composite_{prefix}` text element added | Not implemented |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| Quaternion | `[w, x, y, z]` (4-element array) | `Vector4<f64>` as `[w, x, y, z]` | Direct port — same layout |
| Body position | `body->pos` (3-element `double[]`) | `MjcfBody.pos: Vector3<f64>` | Direct port |
| Body orientation | `body->quat` (4-element `double[]`) | `MjcfBody.quat: Vector4<f64>` | Direct port |
| Geom fromto | `geom->fromto` (6-element `double[]`) | `MjcfGeom.fromto: Option<[f64; 6]>` | Use `Some([...])` |
| Exclude | `mjsExclude` with body name strings | `MjcfContactExclude { body1, body2 }` | Map body names directly |
| Custom text | `mjsText` with name + data strings | Not supported — `<custom><text>` infrastructure incomplete | Out of scope (DT-166) — metadata-only, no physics impact |
| Geom defaults | `def[0].spec` applied via `mjs_addGeom(body, &def[0].spec)` | Parse `<geom>` child as template, apply to each generated geom | Copy parsed geom attributes to each generated `MjcfGeom` |
| Joint defaults | `defjoint[JOINT][0].spec` | Parse `<joint kind="main">` as template | Copy to each generated `MjcfJoint` |

---

## Architecture Decisions

### AD-1: Expansion stage

**Problem:** Where in the parse→build pipeline should composite expansion
happen?

**Alternatives:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | XML pre-processing (before parse) | Clean separation; parser never sees composite | Requires XML manipulation; loses type safety |
| 2 | Parse-time expansion (during parse) | Single pass; no intermediate storage | Mixes parsing and generation; harder to test |
| 3 | Builder-time expansion (after parse, before build) | Type-safe MjcfBody manipulation; follows `expand_frames()` pattern; testable | Requires `MjcfComposite` intermediate type |

**Chosen:** Option 3 — Follows the established `expand_frames()` architecture.
Composites are parsed into `MjcfComposite` structs on `MjcfBody`, then expanded
into `MjcfBody` children during the builder phase. This runs after frame
expansion and before discardvisual/fusestatic (which need to see the expanded
bodies). Matches MuJoCo's approach where `Make()` runs during model compilation,
not XML parsing.

**Frame interaction:** Composites inside `<body>` elements that are inside
`<frame>` elements work correctly: frame expansion resolves frames into bodies
first, then composite expansion finds composites on those bodies. However,
`<composite>` as a **direct child** of `<frame>` (not wrapped in `<body>`)
is not supported — this would require frame parser changes and is not a
standard MuJoCo pattern. Composite parsing is added to `parse_body()` and
`parse_worldbody()` only.

### AD-2: Composite storage on MjcfBody

**Problem:** Where to store parsed composite data before expansion?

**Chosen:** Add `composites: Vec<MjcfComposite>` field to `MjcfBody`. This is
analogous to `frames: Vec<MjcfFrame>` — both are intermediate representations
that get expanded into child bodies during the builder phase. After expansion,
the `composites` vec is empty.

---

## Specification

### S1. Types — Composite data structures

**File:** `sim/L0/mjcf/src/types.rs`
**MuJoCo equivalent:** `mjCComposite` class in `user_composite.h`
**Design decision:** Define Rust enums and structs matching the MuJoCo
composite data model. Only the fields needed for cable generation are
included; deprecated-type-only fields are omitted.

```rust
/// Composite type — only Cable is non-deprecated in MuJoCo 3.4.0.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CompositeType {
    Particle,
    Grid,
    Rope,
    Loop,
    Cable,
    Cloth,
}

/// Curve shape for cable vertex generation.
/// Matches `mjtCompShape` enum in `user_composite.h`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CompositeShape {
    /// v[k] = ix * size[0] / (count[0]-1)
    Line,
    /// v[k] = size[1] * cos(PI * ix * size[2] / (count[0]-1))
    Cos,
    /// v[k] = size[1] * sin(PI * ix * size[2] / (count[0]-1))
    Sin,
    /// v[k] = 0
    #[default]
    Zero,
}

/// Template joint for cable bodies.
#[derive(Debug, Clone)]
pub struct CompositeJoint {
    pub group: Option<i32>,
    pub stiffness: Option<f64>,
    pub damping: Option<f64>,
    pub armature: Option<f64>,
    pub frictionloss: Option<f64>,
    pub limited: Option<bool>,
    pub range: Option<[f64; 2]>,
}

/// Template geom for cable bodies.
#[derive(Debug, Clone)]
pub struct CompositeGeom {
    pub geom_type: MjcfGeomType,
    pub size: [f64; 3],
    pub rgba: Option<[f64; 4]>,
    pub contype: Option<i32>,
    pub conaffinity: Option<i32>,
    pub condim: Option<i32>,
    pub group: Option<i32>,
    pub friction: Option<[f64; 3]>,
    pub mass: Option<f64>,
    pub density: Option<f64>,
    pub solmix: Option<f64>,
    pub solref: Option<[f64; 2]>,
    pub solimp: Option<[f64; 5]>,
    pub margin: Option<f64>,
    pub gap: Option<f64>,
    pub material: Option<String>,
    pub priority: Option<i32>,
}

/// Parsed `<composite>` element.
#[derive(Debug, Clone)]
pub struct MjcfComposite {
    /// Name prefix for generated elements.
    pub prefix: String,
    /// Composite type.
    pub comp_type: CompositeType,
    /// Element count per axis. Cable requires [N, 1, 1] with N >= 2.
    pub count: [i32; 3],
    /// Position offset for first body.
    pub offset: [f64; 3],
    /// Orientation quaternion [w, x, y, z].
    pub quat: [f64; 4],
    /// Initial joint mode for first body: "ball" (default), "free", "none".
    pub initial: String,
    /// Curve shape per axis for vertex generation.
    pub curve: [CompositeShape; 3],
    /// Size parameters: [total_length, amplitude, frequency].
    pub size: [f64; 3],
    /// User-specified vertex positions (alternative to curve generation).
    /// Flat array of [x,y,z, x,y,z, ...].
    pub uservert: Vec<f64>,
    /// Template joint (from `<joint kind="main">` child).
    pub joint: Option<CompositeJoint>,
    /// Template geom (from `<geom>` child).
    pub geom: Option<CompositeGeom>,
}
```

Add `composites: Vec<MjcfComposite>` field to `MjcfBody`:

```rust
pub struct MjcfBody {
    // ... existing fields ...
    /// Composite elements to be expanded during builder phase.
    pub composites: Vec<MjcfComposite>,
}
```

### S2. Parser — Parse `<composite>` element

**File:** `sim/L0/mjcf/src/parser.rs`
**MuJoCo equivalent:** XML parsing in `mjXReader` (not in `user_composite.cc`)
**Design decision:** Add `parse_composite()` function called from both
`parse_body()` and `parse_worldbody()` when a `<composite>` element is
encountered. Replaces the current `skip_element()` behavior.

```rust
fn parse_composite(reader: &mut Reader<&[u8]>, e: &BytesStart) -> Result<MjcfComposite> {
    // Parse attributes
    let prefix = get_attribute_opt(e, "prefix").unwrap_or_default();
    let comp_type = match get_attribute_opt(e, "type")
        .ok_or_else(|| MjcfError::missing_attribute("type", "composite"))?
        .as_str()
    {
        "particle" => CompositeType::Particle,
        "grid" => CompositeType::Grid,
        "rope" => CompositeType::Rope,
        "loop" => CompositeType::Loop,
        "cable" => CompositeType::Cable,
        "cloth" => CompositeType::Cloth,
        other => return Err(MjcfError::invalid_value("type", other, "composite")),
    };

    let count = parse_int_array3_attr(e, "count").unwrap_or([1, 1, 1]);
    let offset = parse_float_array3_attr(e, "offset").unwrap_or([0.0; 3]);
    let quat = parse_float_array4_attr(e, "quat").unwrap_or([1.0, 0.0, 0.0, 0.0]);
    let initial = get_attribute_opt(e, "initial").unwrap_or_else(|| "ball".to_string());
    let size = parse_float_array3_attr(e, "size").unwrap_or([1.0, 0.0, 0.0]);

    // Parse curve attribute: space-separated shape names per axis
    let curve = if let Some(curve_str) = get_attribute_opt(e, "curve") {
        parse_curve_shapes(&curve_str)?
    } else {
        [CompositeShape::Zero; 3]
    };

    // Parse vertex attribute (space-separated float list)
    let uservert = if let Some(v) = get_attribute_opt(e, "vertex") {
        parse_float_list(&v)?
    } else {
        Vec::new()
    };

    // Parse child elements (using existing parser child-element loop pattern).
    // MuJoCo composite supports: <joint kind="main|tendon">, <geom>,
    // <site kind="A|B">, <skin>, <plugin>.
    let mut joint = None;
    let mut geom = None;

    while let Some(child) = read_child_element(reader)? {
        match child.name() {
            "joint" => {
                // kind="main" (default): template for cable body joints.
                // kind="tendon": only used by deprecated cloth type — skip.
                let kind = get_attribute_opt(&child, "kind")
                    .unwrap_or_else(|| "main".to_string());
                if kind == "main" {
                    joint = Some(CompositeJoint {
                        group: parse_int_attr_opt(&child, "group"),
                        stiffness: parse_float_attr_opt(&child, "stiffness"),
                        damping: parse_float_attr_opt(&child, "damping"),
                        armature: parse_float_attr_opt(&child, "armature"),
                        frictionloss: parse_float_attr_opt(&child, "frictionloss"),
                        limited: parse_bool_attr_opt(&child, "limited"),
                        range: parse_float_array2_attr(&child, "range"),
                    });
                }
            }
            "geom" => {
                geom = Some(CompositeGeom {
                    geom_type: get_attribute_opt(&child, "type")
                        .map(|s| parse_geom_type(&s))
                        .transpose()?
                        .unwrap_or(MjcfGeomType::Capsule),
                    size: parse_float_array3_attr(&child, "size")
                        .unwrap_or([0.005, 0.0, 0.0]),
                    rgba: parse_float_array4_attr(&child, "rgba"),
                    contype: parse_int_attr_opt(&child, "contype"),
                    conaffinity: parse_int_attr_opt(&child, "conaffinity"),
                    condim: parse_int_attr_opt(&child, "condim"),
                    group: parse_int_attr_opt(&child, "group"),
                    friction: parse_float_array3_attr(&child, "friction"),
                    mass: parse_float_attr_opt(&child, "mass"),
                    density: parse_float_attr_opt(&child, "density"),
                    solmix: parse_float_attr_opt(&child, "solmix"),
                    solref: parse_float_array2_attr(&child, "solref"),
                    solimp: parse_float_array5_attr(&child, "solimp"),
                    margin: parse_float_attr_opt(&child, "margin"),
                    gap: parse_float_attr_opt(&child, "gap"),
                    material: get_attribute_opt(&child, "material"),
                    priority: parse_int_attr_opt(&child, "priority"),
                });
            }
            "site" | "skin" | "plugin" => {
                // site: Boundary site property customization — deferred
                // skin: Rendering-only — deferred (DT-165)
                // plugin: Requires plugin system — deferred (Spec D)
                skip_element(reader)?;
            }
            _ => {
                skip_element(reader)?;
            }
        }
    }

    Ok(MjcfComposite {
        prefix, comp_type, count, offset, quat, initial,
        curve, size, uservert, joint, geom,
    })
}

fn parse_curve_shapes(s: &str) -> Result<[CompositeShape; 3]> {
    let mut shapes = [CompositeShape::Zero; 3];
    for (i, token) in s.split_whitespace().enumerate() {
        if i >= 3 { break; }
        shapes[i] = match token {
            "s" => CompositeShape::Sin,
            "c" => CompositeShape::Cos,
            "l" => CompositeShape::Line,
            "0" | "zero" => CompositeShape::Zero,
            other => return Err(MjcfError::invalid_value("curve", other, "composite")),
        };
    }
    Ok(shapes)
}
```

In `parse_body()` and `parse_worldbody()`, add handling for `"composite"`:
```rust
"composite" => {
    let composite = parse_composite(reader, &e)?;
    body.composites.push(composite);
}
```

### S3. Cable expansion — Core generation logic

**File:** `sim/L0/mjcf/src/builder/composite.rs` (new file)
**MuJoCo equivalent:** `MakeCable()` + `AddCableBody()` + `mjuu_updateFrame()`
**Design decision:** Self-contained module following `builder/frame.rs` pattern.
Public function `expand_composites()` walks the body tree recursively and
replaces `MjcfComposite` entries with generated `MjcfBody` children. Excludes
are accumulated and returned to the caller for insertion into
`MjcfModel.contact.excludes`.

**Core algorithm:**

```rust
use nalgebra::{Vector3, Vector4, UnitQuaternion};
use std::f64::consts::PI;

/// Expand all composite elements in the body tree.
/// Returns generated contact exclude pairs, or error for deprecated/invalid types.
pub fn expand_composites(body: &mut MjcfBody) -> Result<Vec<MjcfContactExclude>> {
    let mut excludes = Vec::new();
    expand_composites_recursive(body, &mut excludes)?;
    Ok(excludes)
}

fn expand_composites_recursive(
    body: &mut MjcfBody,
    excludes: &mut Vec<MjcfContactExclude>,
) -> Result<()> {
    // First recurse into children
    for child in &mut body.children {
        expand_composites_recursive(child, excludes)?;
    }

    // Then expand composites on this body
    let composites = std::mem::take(&mut body.composites);
    for composite in composites {
        match composite.comp_type {
            CompositeType::Cable => {
                let (bodies, excl) = make_cable(&composite)?;
                body.children.extend(bodies);
                excludes.extend(excl);
            }
            CompositeType::Particle => {
                return Err(MjcfError::composite_error(
                    "The \"particle\" composite type is deprecated. \
                     Please use \"replicate\" instead."
                ));
            }
            CompositeType::Grid => {
                return Err(MjcfError::composite_error(
                    "The \"grid\" composite type is deprecated. \
                     Please use \"flex\" instead."
                ));
            }
            CompositeType::Rope => {
                return Err(MjcfError::composite_error(
                    "The \"rope\" composite type is deprecated. \
                     Please use \"cable\" instead."
                ));
            }
            CompositeType::Loop => {
                return Err(MjcfError::composite_error(
                    "The \"loop\" composite type is deprecated. \
                     Please use \"flexcomp\" instead."
                ));
            }
            CompositeType::Cloth => {
                return Err(MjcfError::composite_error(
                    "The \"cloth\" composite type is deprecated. \
                     Please use \"shell\" instead."
                ));
            }
        }
    }
    Ok(())
}
```

**Vertex generation (matching `MakeCable()` lines 262–288):**

```rust
fn generate_vertices(composite: &MjcfComposite) -> Vec<[f64; 3]> {
    if !composite.uservert.is_empty() {
        // User-specified vertices: chunks of 3
        return composite.uservert.chunks(3)
            .map(|c| [c[0], c[1], c[2]])
            .collect();
    }

    let n = composite.count[0] as usize;
    let quat = UnitQuaternion::from_quaternion(
        nalgebra::Quaternion::new(
            composite.quat[0], composite.quat[1],
            composite.quat[2], composite.quat[3],
        )
    );

    (0..n).map(|ix| {
        let mut v = [0.0f64; 3];
        for k in 0..3 {
            v[k] = match composite.curve[k] {
                CompositeShape::Line => {
                    ix as f64 * composite.size[0] / (n as f64 - 1.0)
                }
                CompositeShape::Cos => {
                    composite.size[1] * (PI * ix as f64 * composite.size[2]
                        / (n as f64 - 1.0)).cos()
                }
                CompositeShape::Sin => {
                    composite.size[1] * (PI * ix as f64 * composite.size[2]
                        / (n as f64 - 1.0)).sin()
                }
                CompositeShape::Zero => 0.0,
            };
        }
        // Rotate by composite quaternion
        let point = Vector3::new(v[0], v[1], v[2]);
        let rotated = quat * point;
        [rotated.x, rotated.y, rotated.z]
    }).collect()
}
```

**Frame computation (matching `mjuu_updateFrame()`):**

```rust
/// Discrete Bishop frame: compute quaternion orientation for a cable segment.
/// Returns (quaternion, edge_length).
fn update_frame(
    normal: &mut Vector3<f64>,
    edge: &Vector3<f64>,
    tprev: &Vector3<f64>,
    tnext: &Vector3<f64>,
    first: bool,
) -> (UnitQuaternion<f64>, f64) {
    let length = edge.norm();
    let tangent = if length > 1e-10 { edge / length } else { Vector3::x() };

    if first {
        // Initialize from tangent and next tangent
        let mut binormal = tangent.cross(tnext);
        let bn = binormal.norm();
        if bn > 1e-10 { binormal /= bn; } else { binormal = Vector3::zeros(); }
        *normal = binormal.cross(&tangent);
        let nn = normal.norm();
        if nn > 1e-10 { *normal /= nn; } else { *normal = Vector3::zeros(); }
    } else {
        // Darboux rotation: rotate normal about vertex binormal
        let mut binormal_axis = tprev.cross(&tangent);
        let ba_norm = binormal_axis.norm();
        let angle = ba_norm.atan2(tprev.dot(&tangent));
        if ba_norm > 1e-10 {
            binormal_axis /= ba_norm;
            let rot = UnitQuaternion::from_axis_angle(
                &nalgebra::Unit::new_unchecked(binormal_axis), angle
            );
            *normal = rot * *normal;
        }
        let nn = normal.norm();
        if nn > 1e-10 { *normal /= nn; }
    }

    let binormal = tangent.cross(normal);
    let bn = binormal.norm();
    let binormal = if bn > 1e-10 { binormal / bn } else { binormal };

    // Construct rotation matrix from frame vectors and convert to quaternion
    let rot_matrix = nalgebra::Matrix3::from_columns(&[tangent, *normal, binormal]);
    let quat = UnitQuaternion::from_rotation_matrix(
        &nalgebra::Rotation3::from_matrix_unchecked(rot_matrix)
    );

    (quat, length)
}
```

**Body chain generation (matching `AddCableBody()`):**

```rust
/// Validate cable composite constraints before generation.
/// Matches MuJoCo's Make() (user_composite.cc:131-193) and
/// MakeCable() (user_composite.cc:245-253) validation.
fn validate_cable(composite: &MjcfComposite) -> Result<()> {
    // --- Make()-level validations (before type dispatch) ---

    // Uservert/count mutual exclusion (Make:147-153).
    // If uservert is non-empty, count[0] must be 1 (the default).
    // MuJoCo overwrites count[0] = uservert.size()/3.
    if !composite.uservert.is_empty() && composite.count[0] != 1 {
        return Err(MjcfError::composite_error(
            "Cannot specify both vertex and count for composite"
        ));
    }

    // Uservert length must be a multiple of 3 (xyz triples)
    if !composite.uservert.is_empty() && composite.uservert.len() % 3 != 0 {
        return Err(MjcfError::composite_error(
            "Composite vertex data must have length divisible by 3"
        ));
    }

    // Effective vertex count: uservert overrides count[0] (Make:152-153)
    let effective_count0 = if !composite.uservert.is_empty() {
        (composite.uservert.len() / 3) as i32
    } else {
        composite.count[0]
    };

    // Size validation (Make:142-144) — only when no uservert.
    // MuJoCo checks: mju_dot3(size, size) >= mjMINVAL
    if composite.uservert.is_empty() {
        let size_sq = composite.size[0] * composite.size[0]
            + composite.size[1] * composite.size[1]
            + composite.size[2] * composite.size[2];
        if size_sq < 1e-15 {
            return Err(MjcfError::composite_error(
                "Composite size is too small"
            ));
        }
    }

    // Initial value validation — MuJoCo accepts "ball", "none", "free"
    match composite.initial.as_str() {
        "ball" | "none" | "free" => {}
        other => {
            return Err(MjcfError::composite_error(
                &format!(
                    "Invalid composite initial value: \"{other}\". \
                     Must be \"ball\", \"none\", or \"free\""
                )
            ));
        }
    }

    // --- MakeCable()-level validations ---

    // Cable must be 1D: count[1] and count[2] must be 1 (MakeCable:245)
    if composite.count[1] != 1 || composite.count[2] != 1 {
        return Err(MjcfError::composite_error("Cable must be one-dimensional"));
    }

    // Need at least 2 vertices (1 segment)
    if effective_count0 < 2 {
        return Err(MjcfError::composite_error(
            "Cable requires count >= 2 or at least 2 user-specified vertices"
        ));
    }

    // Validate geom type: must be cylinder, capsule, or box (MakeCable:250-253).
    // Note: MuJoCo's error message says "sphere" but the code validates
    // for cylinder (user_composite.cc:250). We match the actual behavior
    // (accept cylinder/capsule/box) and use MuJoCo's exact error text.
    if let Some(ref geom) = composite.geom {
        match geom.geom_type {
            MjcfGeomType::Cylinder | MjcfGeomType::Capsule | MjcfGeomType::Box => {}
            _ => {
                return Err(MjcfError::composite_error(
                    "Cable geom type must be sphere, capsule or box"
                ));
            }
        }
    } else {
        return Err(MjcfError::composite_error(
            "Cable composite requires a <geom> child element"
        ));
    }

    Ok(())
}

fn make_cable(composite: &MjcfComposite) -> Result<(Vec<MjcfBody>, Vec<MjcfContactExclude>)> {
    validate_cable(composite)?;

    let verts = generate_vertices(composite);
    let n = verts.len();  // count[0]
    let num_bodies = n - 1;  // count[0] - 1
    let prefix = &composite.prefix;

    let mut bodies: Vec<MjcfBody> = Vec::new();
    let mut excludes: Vec<MjcfContactExclude> = Vec::new();
    let mut normal = Vector3::new(0.0, 1.0, 0.0);
    let mut prev_quat = UnitQuaternion::identity();

    for ix in 0..num_bodies {
        let last_idx = num_bodies - 1;
        let is_first = ix == 0;
        let is_last = ix == last_idx;

        // Compute edge and tangent vectors
        let edge = Vector3::new(
            verts[ix+1][0] - verts[ix][0],
            verts[ix+1][1] - verts[ix][1],
            verts[ix+1][2] - verts[ix][2],
        );

        let mut tprev = Vector3::zeros();
        let mut length_prev = 0.0;
        if !is_first {
            tprev = Vector3::new(
                verts[ix][0] - verts[ix-1][0],
                verts[ix][1] - verts[ix-1][1],
                verts[ix][2] - verts[ix-1][2],
            );
            length_prev = tprev.norm();
            if length_prev > 1e-10 { tprev /= length_prev; }
        }

        let mut tnext = Vector3::zeros();
        if !is_last {
            tnext = Vector3::new(
                verts[ix+2][0] - verts[ix+1][0],
                verts[ix+2][1] - verts[ix+1][1],
                verts[ix+2][2] - verts[ix+1][2],
            );
            let tn = tnext.norm();
            if tn > 1e-10 { tnext /= tn; }
        }

        // Update moving frame
        let (this_quat, length) = update_frame(
            &mut normal, &edge, &tprev, &tnext, is_first,
        );

        // Body, joint, geom names
        let body_name = match (is_first, is_last) {
            (true, _) => format!("{prefix}B_first"),
            (_, true) => format!("{prefix}B_last"),
            _ => format!("{prefix}B_{ix}"),
        };
        let next_body_name = if is_last {
            format!("{prefix}B_first")
        } else if ix + 1 == last_idx {
            format!("{prefix}B_last")
        } else {
            format!("{prefix}B_{}", ix + 1)
        };
        let joint_name = match (is_first, is_last) {
            (true, _) => format!("{prefix}J_first"),
            (_, true) => format!("{prefix}J_last"),
            _ => format!("{prefix}J_{ix}"),
        };
        let geom_name = format!("{prefix}G{ix}");

        // Body position and orientation
        let (body_pos, body_quat) = if is_first {
            let pos = Vector3::new(
                composite.offset[0] + verts[ix][0],
                composite.offset[1] + verts[ix][1],
                composite.offset[2] + verts[ix][2],
            );
            (pos, Vector4::new(this_quat.w, this_quat.i, this_quat.j, this_quat.k))
        } else {
            let pos = Vector3::new(length_prev, 0.0, 0.0);
            let dquat = prev_quat.inverse() * this_quat;
            (pos, Vector4::new(dquat.w, dquat.i, dquat.j, dquat.k))
        };

        // Build MjcfGeom from template
        let geom = build_cable_geom(composite, &geom_name, length);

        // Build MjcfJoint
        let joint = build_cable_joint(composite, &joint_name, is_first);

        // Build sites — first and last body get boundary sites.
        // MuJoCo uses two separate `if` checks (AddCableBody:436-442),
        // NOT if/else. For count=2 the single body is both first AND last,
        // so it gets BOTH S_first and S_last.
        let mut sites = Vec::new();
        if is_first {
            sites.push(MjcfSite {
                name: format!("{prefix}S_first"),
                pos: Vector3::zeros(),
                ..Default::default()
            });
        }
        if is_last {
            sites.push(MjcfSite {
                name: format!("{prefix}S_last"),
                pos: Vector3::new(length, 0.0, 0.0),
                ..Default::default()
            });
        }

        // Assemble body
        let mut body = MjcfBody {
            name: body_name.clone(),
            pos: body_pos,
            quat: body_quat,
            geoms: vec![geom],
            joints: joint.into_iter().collect(),
            sites,
            ..Default::default()
        };

        // Contact exclude for all except last
        if !is_last {
            excludes.push(MjcfContactExclude {
                name: None,
                body1: body_name.clone(),
                body2: next_body_name,
            });
        }

        prev_quat = this_quat;

        // Chain: each body is a child of the previous
        if let Some(last_body) = bodies.last_mut() {
            // Move the new body into the last body's children
            // (creates linear chain: B_first -> B_1 -> ... -> B_last)
            fn append_to_chain(parent: &mut MjcfBody, child: MjcfBody) {
                if parent.children.is_empty() {
                    parent.children.push(child);
                } else {
                    let last = parent.children.last_mut().unwrap();
                    append_to_chain(last, child);
                }
            }
            append_to_chain(last_body, body);
        } else {
            bodies.push(body);
        }
    }

    Ok((bodies, excludes))
}
```

Note: The chain construction appends each new body as a child of the deepest
leaf, producing the linear chain `B_first → B_1 → ... → B_last`. The
`bodies` vec will contain exactly one element (the root `B_first`) with nested
children.

**Helper — build cable geom:**

```rust
fn build_cable_geom(composite: &MjcfComposite, name: &str, length: f64) -> MjcfGeom {
    // geom presence validated by validate_cable() before this point
    let template = composite.geom.as_ref().unwrap();

    // Copy ALL template fields explicitly — no elision.
    // MuJoCo applies def[0].spec as the geom default, then overrides
    // fromto/pos/size based on geom type. We replicate by copying the
    // parsed <geom> child attributes as-is, then setting placement.
    let mut geom = MjcfGeom {
        name: name.to_string(),
        geom_type: template.geom_type,
        size: template.size,
        rgba: template.rgba,
        contype: template.contype,
        conaffinity: template.conaffinity,
        condim: template.condim,
        group: template.group,
        friction: template.friction,
        mass: template.mass,
        density: template.density,
        solmix: template.solmix,
        solref: template.solref,
        solimp: template.solimp,
        margin: template.margin,
        gap: template.gap,
        material: template.material.clone(),
        priority: template.priority,
        ..Default::default()
    };

    // Override placement based on geom type (AddCableBody:392-403)
    match template.geom_type {
        MjcfGeomType::Capsule | MjcfGeomType::Cylinder => {
            // fromto along local x-axis: [0,0,0] → [length,0,0]
            geom.fromto = Some([0.0, 0.0, 0.0, length, 0.0, 0.0]);
        }
        MjcfGeomType::Box => {
            // Centered at midpoint, half-length along x
            geom.pos = Vector3::new(length / 2.0, 0.0, 0.0);
            geom.size[0] = length / 2.0;
        }
        _ => {} // Validated earlier — unreachable
    }
    geom
}
```

**Helper — build cable joint:**

```rust
fn build_cable_joint(
    composite: &MjcfComposite,
    name: &str,
    is_first: bool,
) -> Option<MjcfJoint> {
    // No joint if first body and initial="none"
    if is_first && composite.initial == "none" {
        return None;
    }

    let template = composite.joint.as_ref();
    let is_free = is_first && composite.initial == "free";

    let jnt_type = if is_free { MjcfJointType::Free } else { MjcfJointType::Ball };

    // Free joints get zero damping/armature/frictionloss
    let (damping, armature, frictionloss) = if is_free {
        (Some(0.0), Some(0.0), Some(0.0))
    } else {
        (
            template.and_then(|t| t.damping),
            template.and_then(|t| t.armature),
            template.and_then(|t| t.frictionloss),
        )
    };

    Some(MjcfJoint {
        name: name.to_string(),
        joint_type: jnt_type,
        damping,
        armature,
        frictionloss,
        group: template.and_then(|t| t.group),
        stiffness: template.and_then(|t| t.stiffness),
        limited: template.and_then(|t| t.limited),
        range: template.and_then(|t| t.range),
        ..Default::default()
    })
}
```

### S4. Pipeline integration

**File:** `sim/L0/mjcf/src/builder/mod.rs`
**MuJoCo equivalent:** `mjCComposite::Make()` called from `mjCModel::Compile()`
**Design decision:** Call `expand_composites()` after `expand_frames()` and
before `discardvisual`/`fusestatic`. This matches MuJoCo's order: frames are
expanded first (so composites inside frames work), then composites are
expanded, then compiler passes see the fully expanded body tree.

```rust
// In model_from_mjcf():

// Expand <frame> elements first
expand_frames(&mut mjcf.worldbody, &mjcf.compiler, None);

// Expand <composite> elements — after frames so composites inside frames work
let composite_excludes = composite::expand_composites(&mut mjcf.worldbody)?;
mjcf.contact.excludes.extend(composite_excludes);

// Compiler passes see fully expanded body tree
if mjcf.compiler.discardvisual {
    apply_discardvisual(&mut mjcf);
}
```

Register the module in `builder/mod.rs`:
```rust
mod composite;
```

---

## Acceptance Criteria

### AC1: Cable body count *(runtime test — MuJoCo-verified)*
**Given:** `<composite type="cable" count="5 1 1" curve="s 0 0" size="1" offset="0 0 1">` with capsule geom and main joint
**After:** Model loading
**Assert:** `nbody == 5` (world + 4 cable bodies), `njnt == 4`, `ngeom == 4`
**Field:** `Model.nbody`, `Model.njnt`, `Model.ngeom`

### AC2: Cable body naming *(runtime test — MuJoCo-verified)*
**Given:** Same model as AC1
**After:** Model loading
**Assert:** Body names are `B_first`, `B_1`, `B_2`, `B_last` in order
**Field:** Body name lookup

### AC3: Cable body chain *(runtime test — MuJoCo-verified)*
**Given:** Same model as AC1
**After:** Model loading
**Assert:** Body parent chain: `B_first→world`, `B_1→B_first`, `B_2→B_1`, `B_last→B_2`
**Field:** `Model.body_parentid`

### AC4: Cable with initial="none" *(runtime test — MuJoCo-verified)*
**Given:** Cable count=4, `initial="none"`
**After:** Model loading
**Assert:** `njnt == 2` (no joint on B_first, ball joints on B_1 and B_last)
**Field:** `Model.njnt`, `Model.jnt_type`

### AC5: Cable with initial="free" *(runtime test — MuJoCo-verified)*
**Given:** Cable count=4, `initial="free"`
**After:** Model loading
**Assert:** `njnt == 3`, first joint is free (type=0), others are ball (type=1). Free joint has damping=0.
**Field:** `Model.njnt`, `Model.jnt_type`, `Model.jnt_damping`

### AC6: Deprecated type errors *(runtime test — MuJoCo-verified)*
**Given:** `<composite type="rope">`, `type="grid"`, `type="particle"`, `type="loop"`, `type="cloth"`
**After:** Model loading attempt
**Assert:** Each returns error containing the MuJoCo-conformant deprecation message (per EGT-3)
**Field:** Error message content

### AC7: Cable with prefix *(runtime test — MuJoCo-verified)*
**Given:** Cable with `prefix="R"`, count=3
**After:** Model loading
**Assert:** Body names are `RB_first`, `RB_last`. Joint names are `RJ_first`, `RJ_last`. Geom names are `RG0`, `RG1`.
**Field:** Element name lookup

### AC8: Contact excludes *(runtime test — MuJoCo-verified)*
**Given:** Cable count=5 (4 cable bodies)
**After:** Model loading
**Assert:** 3 contact exclude pairs for adjacent bodies
**Field:** `Model.exclude_signature` or exclude body pairs

### AC9: Boundary sites *(runtime test — MuJoCo-verified)*
**Given:** Cable count=4
**After:** Model loading
**Assert:** Exactly 2 sites: `S_first` on first body at `[0,0,0]`, `S_last` on last body at `[edge_length, 0, 0]`
**Field:** `Model.site_pos`, `Model.site_bodyid`

### AC10: Geom placement *(runtime test — MuJoCo-verified)*
**Given:** Cable count=4, capsule geom with `size=".005"`
**After:** Model loading
**Assert:** Each geom is capsule type with `fromto = [0, 0, 0, edge_length, 0, 0]` and `size[0] = 0.005`
**Field:** `Model.geom_type`, `Model.geom_size`

### AC11: Pipeline ordering *(code review)*
**Assert:** `expand_composites()` is called after `expand_frames()` and before `apply_discardvisual()`/`apply_fusestatic()` in `model_from_mjcf()`.

### AC12: Nested body composite *(runtime test — MuJoCo-verified)*
**Given:** Cable inside a `<body name="parent" pos="1 0 0">` element
**After:** Model loading
**Assert:** `B_first` is a child of `parent` body (not world)
**Field:** `Model.body_parentid`

### AC13: Count=2 minimum cable *(runtime test — MuJoCo-verified)*
**Given:** Cable count=2 (minimum)
**After:** Model loading
**Assert:** `nbody == 2` (world + 1 cable body), `njnt == 1`, `ngeom == 1`, `nexclude == 0`, `nsite == 2` (both `S_first` and `S_last` on same body — MuJoCo uses two separate `if` checks, not `if/else`, so count=2 body is both first AND last)
**Field:** Element counts, `Model.site_bodyid`

### AC14: Invalid geom type *(runtime test — MuJoCo-verified)*
**Given:** Cable with `<geom type="sphere">`
**After:** Model loading attempt
**Assert:** Error: cable geom type must be capsule, cylinder, or box
**Field:** Error message

### AC15: Uservert cable *(runtime test — MuJoCo-verified)*
**Given:** Cable with `vertex="0 0 0 1 0 0 2 0 1"` (3 vertices), no `count` attribute, capsule geom
**After:** Model loading
**Assert:** `nbody == 3` (world + 2 cable bodies), `njnt == 2`, body positions reflect user-specified vertex positions (not curve-generated)
**Field:** `Model.nbody`, `Model.body_pos`

### AC16: Uservert/count mutual exclusion *(runtime test — MuJoCo-verified)*
**Given:** Cable with both `count="5 1 1"` AND `vertex="0 0 0 1 0 0"`
**After:** Model loading attempt
**Assert:** Error: "Cannot specify both vertex and count for composite"
**Field:** Error message

### AC17: Count validation errors *(runtime test)*
**Given:** (a) Cable with `count="1 1 1"` (too few), (b) Cable with `count="5 5 1"` (multi-dimensional)
**After:** Model loading attempt
**Assert:** (a) Error about count >= 2, (b) Error about cable being one-dimensional
**Field:** Error messages

### AC18: Invalid initial value *(runtime test)*
**Given:** Cable with `initial="xyz"`
**After:** Model loading attempt
**Assert:** Error about invalid initial value
**Field:** Error message

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (body count) | T1 | Direct |
| AC2 (naming) | T1 | Direct |
| AC3 (chain) | T1 | Direct |
| AC4 (initial=none) | T2 | Direct |
| AC5 (initial=free) | T3 | Direct |
| AC6 (deprecated) | T4 | Direct |
| AC7 (prefix) | T5 | Direct |
| AC8 (excludes) | T1 | Direct |
| AC9 (sites) | T1 | Direct |
| AC10 (geom placement) | T1, T10 | Direct |
| AC11 (pipeline order) | — | Code review |
| AC12 (nested) | T6 | Direct |
| AC13 (count=2 + dual sites) | T7 | Edge case |
| AC14 (invalid geom) | T8 | Error case |
| AC15 (uservert cable) | T11 | Direct |
| AC16 (uservert/count exclusion) | T12 | Error case |
| AC17 (count validation) | T12 | Error case |
| AC18 (invalid initial) | T12 | Error case |

---

## Test Plan

### T1: Cable basic generation → AC1, AC2, AC3, AC8, AC9, AC10
Cable with count=5, curve="s 0 0", size="1", capsule geom. Verify body count
(5), names (B_first/B_1/B_2/B_last), parent chain, 3 excludes, 2 sites,
geom fromto placement. MuJoCo 3.4.0 verified.

### T2: Cable initial="none" → AC4
Cable count=4 with initial="none". Verify first body has no joint, others
have ball joints. njnt=2.

### T3: Cable initial="free" → AC5
Cable count=4 with initial="free". Verify first body has free joint with
damping=0, others have ball joints. njnt=3.

### T4: Deprecated type errors → AC6
Test each deprecated type (particle, grid, rope, loop, cloth). Verify error
message matches MuJoCo's exact deprecation text.

### T5: Cable with prefix → AC7
Cable with prefix="R", count=3. Verify all element names start with "R":
RB_first, RB_last, RJ_first, RJ_last, RG0, RG1.

### T6: Cable in nested body → AC12
Cable inside `<body name="parent" pos="1 0 0">`. Verify B_first's parent
is "parent", not world.

### T7: Cable minimum count (+ dual sites) → AC13
Cable count=2. Verify single body (name `B_first`), single joint, single
geom, zero excludes, **two sites** (`S_first` at `[0,0,0]` AND `S_last` at
`[edge_length, 0, 0]`) on the same body. This is the critical count=2 edge
case where MuJoCo's two separate `if` checks (not `if/else`) put both
boundary sites on the single body.

### T8: Invalid geom type → AC14
Cable with `<geom type="sphere">`. Verify error about invalid geom type.

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| count=2 (minimum cable) | Single body with dual sites, no excludes | T7 | AC13 |
| count=1 (too few) | Must error: need at least 2 vertices | T12 | AC17 |
| count=[5,5,1] (multi-dim) | Must error: cable is 1D only | T12 | AC17 |
| initial="none" (pinned) | No joint on first body | T2 | AC4 |
| initial="free" (6-DOF root) | Free joint with zero damping | T3 | AC5 |
| initial="xyz" (invalid) | Must error: unknown initial value | T12 | AC18 |
| Deprecated types (5) | Must match MuJoCo error messages | T4 | AC6 |
| Prefix naming | All elements get prefix | T5 | AC7 |
| Nested body parent | Composite inside body, not worldbody | T6 | AC12 |
| Invalid geom type | sphere/ellipsoid rejected | T8 | AC14 |
| Multiple composites | Two cables with different prefixes coexist | T9 | — |
| Uservert cable | User-specified vertices instead of curve | T11 | AC15 |
| Uservert + count conflict | Both specified → error | T12 | AC16 |
| Zero size (no uservert) | Degenerate cable → error | T12 | — |

### T10: Cable with cylinder geom → AC10
Cable with `<geom type="cylinder" size=".005">`. Verify geoms are cylinder
type with fromto placement `[0,0,0, edge_length,0,0]`.

### T11: Cable with user-specified vertices → AC15
Cable with `vertex="0 0 0 1 0 0 2 0 1"` (3 vertices, no count attribute).
Verify 2 cable bodies, body positions reflect the user vertex positions,
not curve-generated values.

### T12: Validation error cases → AC16, AC17, AC18
Multiple sub-cases in one test function:
- (a) `count="1 1 1"` → error about count >= 2
- (b) `count="5 5 1"` → error about cable being one-dimensional
- (c) `count="5 1 1"` + `vertex="0 0 0 1 0 0"` → error about mutual exclusion
- (d) `initial="xyz"` → error about invalid initial value
- (e) `size="0 0 0"` (no uservert) → error about size too small

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| T9: Multiple composites | Two cables (prefix A and B) in same model | Verifies no name collision and both chains generated correctly |
| T11: Uservert cable | User-specified vertices (distinct code path from curve generation) | Exercises `generate_vertices()` uservert branch |
| T12: Validation errors | Make()-level and MakeCable()-level validation | Exercises all error paths in `validate_cable()` |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| `<composite>` parsing | Silently skipped | Parsed and expanded for cable; error for deprecated types | Toward MuJoCo | Models using `<composite>` | None — transparent improvement |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `sim/L0/mjcf/src/types.rs` | Add `MjcfComposite`, `CompositeType`, `CompositeShape`, `CompositeJoint`, `CompositeGeom` types; add `composites` field to `MjcfBody` | +120 |
| `sim/L0/mjcf/src/parser.rs` | Add `parse_composite()`, `parse_curve_shapes()`, child element parsing, call from `parse_body()`/`parse_worldbody()` | +130 |
| `sim/L0/mjcf/src/builder/composite.rs` | New file: `expand_composites()`, `make_cable()`, `validate_cable()`, `generate_vertices()`, `update_frame()`, helpers | +400 |
| `sim/L0/mjcf/src/builder/mod.rs` | Add `mod composite;` and `expand_composites()` call after `expand_frames()` | +5 |
| `sim/L0/mjcf/src/lib.rs` | Update module exports, remove "composite not supported" doc note | ~5 |
| `sim/L0/mjcf/tests/` or `sim/L0/tests/` | New test file for composite tests | +300 |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| All existing 2,273+ tests | Various | Pass (unchanged) | `<composite>` was previously skipped; models without composite are unaffected |
| `composite_model` conformance test | `sim/L0/tests/` | Pass (unchanged) | Named "composite" but uses explicit bodies, not `<composite>` element |

### Non-Modification Sites

| File:line | What it does | Why NOT modified |
|-----------|-------------|-----------------|
| `builder/body.rs` | Body tree processing | Sees expanded bodies — no composite awareness needed |
| `builder/joint.rs` | Joint processing | Sees expanded joints — no change needed |
| `builder/geom.rs` | Geom processing | Sees expanded geoms — no change needed |
| `builder/contact.rs` | Exclude processing | Already handles `MjcfContactExclude` from parser — composite excludes use same path |

---

## Execution Order

1. **S1** (types) first — all other sections depend on the type definitions
2. **S2** (parser) after S1 — parsing produces `MjcfComposite` structs
3. **S3** (cable expansion) after S1 — expansion consumes `MjcfComposite`, produces `MjcfBody`
4. **S4** (pipeline integration) after S2+S3 — wires parsing and expansion together

S2 and S3 are independent of each other and could be implemented in parallel,
but sequential implementation (S2→S3) allows testing parsing before expansion.

After each section: `cargo test -p sim-mjcf` to verify no regressions.
After S4: full integration test with cable MJCF models.

---

## Out of Scope

- **Skin generation** — Rendering-only feature for box-geom cables. Complex bicubic
  interpolation with subgrid support. Tracked as DT-165. Conformance impact: none
  (skin is visual-only, no physics effect).
- **Plugin support** — `mujoco.elasticity.cable` plugin requires Spec D (plugin system).
  Deferred to post-Spec-D. Conformance impact: models using cable plugins will error
  during expansion; models without plugins work.
- **`<flexcomp>` element** — Separate MJCF element replacing most deprecated composite
  types. Different infrastructure (generates flex bodies). Separate future spec.
- **`<replicate>` element** — Separate MJCF element replacing `particle` composite.
  Separate future spec.
- **Custom text generation** — Cable adds `composite_{prefix}` text to model.
  `<custom><text>` support may not be complete. Tracked as DT-166. Low-priority
  metadata with no physics impact.
- **Cable `<site>` template** — The `<site>` child of `<composite>` customizes
  site properties (group, size, material, rgba). Initial implementation uses
  default site properties. Refinement deferred.
