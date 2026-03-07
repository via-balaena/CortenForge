# Spec B — Cotangent Laplacian Bending: Spec

**Status:** Draft
**Phase:** Roadmap Phase 10 — Flex Pipeline
**Effort:** L
**MuJoCo ref:** `ComputeBending()` in `user_mesh.cc`, lines ~3666–3712;
`cot()` in `user_mesh.cc`, lines ~3645–3654; `ComputeVolume()` in
`user_mesh.cc`, lines ~3655–3663; `CreateFlapStencil()` in `user_mesh.cc`,
lines ~3593–3639; runtime bending loop in `engine_passive.c`, lines ~192–268
**MuJoCo version:** 3.5.0
**Test baseline:** 1,900+ sim domain tests
**Prerequisites:**
- T1 Session 2 (commit `1f0230c`): provides `flex_rigid`, `flexedge_rigid`,
  `flexedge_length`, `flexedge_velocity`
- Spec A (commit `0a46fef`): provides `flexedge_J` sparse Jacobian (soft dep
  only — not consumed by Spec B for free vertices)

**Independence:** This spec is independent of Spec A (§42A-i edge Jacobian)
and Spec C/D (§42A-iv/v collision) per the umbrella dependency graph. Shared
file `passive.rs` is modified by Spec A (edge spring force projection, lines
~533–545) and Spec B (bending rewrite, lines ~548–670) — at different code
sections. No conflict.

> **Conformance mandate (split):** This spec has a split conformance posture:
> 1. **Cotangent Laplacian bending** (S1–S6) — MUST produce numerically
>    identical results to MuJoCo's `ComputeBending()` and the bending section
>    of `engine_passive.c`. The MuJoCo C source code is the source of truth.
> 2. **`FlexBendingModel` trait, `BridsonBending` impl, `bending_model` MJCF
>    attribute** (S7–S8) — are CortenForge EXTENSIONS. MuJoCo has no trait
>    boundary, no Bridson option, and no `bending_model` attribute. The
>    existing Bridson implementation and test suite are the source of truth
>    for the extension subset.

---

## Scope Adjustment

| Umbrella claim | MuJoCo reality (C source) | Action |
|----------------|--------------------------|--------|
| `flex_bending`: 17 f64 per edge, flat layout | MuJoCo 3.5.0: `flex_bending` shaped `(nflexedge, 17)` — `bending[17*e + 4*i + j]` for 4x4 matrix, `bending[17*e + 16]` for curved ref. Confirmed. | In scope |
| `flexedge_flap`: 2 vertices per edge | MuJoCo 3.5.0: `flex_edgeflap` shaped `(nflexedge, 2)`. `flap[0]` = opposite vertex in tri 1, `flap[1]` = opposite vertex in tri 2 (`-1` for boundary). Confirmed. | In scope |
| Cotangent weights from rest geometry | `ComputeBending()` uses `cot()` helper. Confirmed. | In scope |
| Material stiffness: `mu = young / (2*(1+poisson))` | `user_mesh.cc:4379`: `mu` passed as `young / (2 * (1 + poisson))`. Stiffness = `3 * mu * thickness^3 / (24 * volume)`. Confirmed. | In scope |
| 4x4 stiffness matrix outer product | `bending[4*i+j] += c[i] * c[j] * cos_theta * stiffness`. Confirmed. | In scope |
| Curved reference (Garg, b[16]) | `bending[16] = dot(n, e2) * (a01-a03) * (a04-a02) * stiffness / (sqr * sqrt(sqr))`. Confirmed. | In scope |
| Runtime: matrix-vector spring + damper | `engine_passive.c:245–253`: triple-nested loop. Confirmed. | In scope |
| `FlexBendingModel` trait | Not in MuJoCo. CortenForge extension. | In scope (extension) |
| `bending_model` MJCF attribute | Not in MuJoCo. CortenForge extension. | In scope (extension) |
| Bending only for `dim == 2` | `engine_passive.c:205`: `if (dim == 2)` gates bending. Confirmed. | In scope |
| Bending gated on `flex_rigid[f]` | `engine_passive.c:201`: `if (dim == 1 || m->flex_rigid[f]) continue;` Confirmed. | In scope |
| `elastic2d` gate for precomputation | `user_mesh.cc:4374`: `if (dim == 2 && (elastic2d == 1 || elastic2d == 3))`. CortenForge always computes bending for `dim == 2` with `thickness > 0` (matching default shell behavior). | In scope — documented in S2 |
| Damper multiplied by `flex_damping[f]` | `engine_passive.c:264`: `qfrc_damper[...] -= damper[...] * flex_damping[f]`. Confirmed. | In scope |
| Force insertion via `body_dofadr` / `body_dofnum` | `engine_passive.c:258–265`: iterates `body_dofnum` DOFs. For free vertices, `body_dofnum == 3`. Confirmed. | In scope (free-vertex only) |

**Final scope:**
1. `flexedge_flap` topology computation from element connectivity
2. Cotangent weight precomputation (Wardetzky operator) from rest geometry
3. Material stiffness computation (shear modulus, plate bending formula)
4. 4x4 stiffness matrix construction (outer product with cos_theta)
5. Curved reference coefficient (Garg correction, b[16])
6. Runtime cotangent force application (spring + damper matrix-vector)
7. `FlexBendingModel` trait definition and dispatch (extension)
8. `BridsonBending` impl — preserve current dihedral code (extension)

---

## Problem Statement

**Conformance gap** — MuJoCo uses a cotangent Laplacian bending model
(Wardetzky et al. "Discrete Quadratic Curvature Energies" with Garg et al.
"Cubic Shells" curved reference correction) to compute bending forces for
`dim=2` flex bodies. The model precomputes a 4x4 stiffness matrix per edge
at build time (`ComputeBending()` in `user_mesh.cc:3666–3712`), then applies
bending forces at runtime via a matrix-vector multiply (`engine_passive.c:
206–268`). This produces **constant** bending stiffness (the B matrix is
pre-computed from rest geometry) with no per-step stability clamp needed.

CortenForge currently implements Bridson dihedral angle springs
(`passive.rs:548–670`), which compute bending forces from the runtime
dihedral angle error with a per-vertex stability clamp. This is a **different
algorithm** that produces different force values for the same input state.
While Bridson is a valid bending model (and is nonlinear / large-deformation
accurate), it does not match MuJoCo's output.

This spec replaces the default bending model with MuJoCo's cotangent
Laplacian while preserving the Bridson implementation behind a
`FlexBendingModel` trait — the first trait boundary in the sim pipeline.

---

## MuJoCo Reference

> **This is the most important section of the spec.** Everything downstream —
> the algorithm, the acceptance criteria, the tests — is derived from what's
> documented here.

### Build-time: `CreateFlapStencil()` topology

**Source:** `user_mesh.cc:3593–3639` (MuJoCo 3.5.0)

Builds the flap topology (opposite vertices for each edge) from element
connectivity. For each triangle, extracts 3 edges using local numbering
`{{1,2}, {2,0}, {0,1}}`. Creates canonical edge key: `(min(va, vb),
max(va, vb))`. First encounter: stores opposite vertex as `flap[0]`.
Second encounter: stores opposite vertex as `flap[1]`. Boundary edges
retain `flap[1] = -1`.

The diamond stencil for edge `e` has 4 vertices:
- `v[0] = flexedge_vert[e][0]` — first edge endpoint
- `v[1] = flexedge_vert[e][1]` — second edge endpoint
- `v[2] = flexedge_flap[e][0]` — opposite vertex in triangle 1
- `v[3] = flexedge_flap[e][1]` — opposite vertex in triangle 2 (`-1` if boundary)

**Vertex ordering symmetry:** Which triangle is encountered first is
arbitrary. The `cot()` computation in `ComputeBending()` is symmetric with
respect to which triangle is "tri 1" vs "tri 2" — swapping them produces
identical weight vectors and stiffness matrix.

### Build-time: `cot()` helper

**Source:** `user_mesh.cc:3645–3654` (MuJoCo 3.5.0)

```c
double inline cot(const double* x, int v0, int v1, int v2) {
    double edge1[3] = {x[3*v1]-x[3*v0], x[3*v1+1]-x[3*v0+1], x[3*v1+2]-x[3*v0+2]};
    double edge2[3] = {x[3*v2]-x[3*v0], x[3*v2+1]-x[3*v0+1], x[3*v2+2]-x[3*v0+2]};
    double normal[3];
    mjuu_crossvec(normal, edge1, edge2);
    return mjuu_dot3(edge1, edge2) / sqrt(mjuu_dot3(normal, normal));
}
```

Geometric interpretation: `cot(angle at v0)` in triangle `(v0, v1, v2)` =
`dot(e1,e2) / |cross(e1,e2)| = cos(alpha) / sin(alpha) = cot(alpha)`.

### Build-time: `ComputeVolume()` helper

**Source:** `user_mesh.cc:3655–3663` (MuJoCo 3.5.0)

```c
double inline ComputeVolume(const double* x, const int v[3]) {
    // area = |cross(edge1, edge2)| / 2
    double edge1[3], edge2[3], normal[3];
    // edge1 = v[1] - v[0], edge2 = v[2] - v[0]
    mjuu_crossvec(normal, edge1, edge2);
    return sqrt(mjuu_dot3(normal, normal)) / 2;
}
```

Returns the **area** of the triangle (not volume). The diamond "volume" is
the sum of the areas of both adjacent triangles:
`volume = ComputeVolume(pos, [v[0], v[1], v[2]]) + ComputeVolume(pos, [v[1], v[0], v[3]])`.

### Build-time: `ComputeBending()` algorithm

**Source:** `user_mesh.cc:3666–3712` (MuJoCo 3.5.0)

For each edge with two adjacent triangles (not a boundary edge), computes
17 coefficients stored in `flex_bending[17*e + 0..17]`.

**Step 1: Four cotangent values:**
```
a01 = cot(v[0], v[1], v[2])  — angle at v[0] in tri 1
a02 = cot(v[0], v[3], v[1])  — angle at v[0] in tri 2
a03 = cot(v[1], v[2], v[0])  — angle at v[1] in tri 1
a04 = cot(v[1], v[0], v[3])  — angle at v[1] in tri 2
```

**Step 2: Weight vector c[0..4]:**
```
c[0] = a03 + a04   (sum of cots at v[1] in both tris)
c[1] = a01 + a02   (sum of cots at v[0] in both tris)
c[2] = -(a01 + a03) (negative sum of cots from tri 1)
c[3] = -(a02 + a04) (negative sum of cots from tri 2)
```

Fundamental property: `c[0] + c[1] + c[2] + c[3] = 0`. This ensures all
row sums of the outer-product matrix are zero (cotangent Laplacian property).

**Step 3: Diamond volume (area):**
```
volume = area(v[0], v[1], v[2]) + area(v[1], v[0], v[3])
```

**Step 4: Material stiffness:**
```
stiffness = 3 * mu * thickness^3 / (24 * volume)
```
where `mu = young / (2 * (1 + poisson))` (shear modulus). The caller passes
`mu` at `user_mesh.cc:4379`.

**Step 5: Edge vectors and transport vectors:**
```c
double e0[3] = v1 - v0;   // shared edge
double e1[3] = v2 - v0;   // to opposite vertex in tri 1
double e2[3] = v3 - v0;   // to opposite vertex in tri 2
double e3[3] = v2 - v1;
double e4[3] = v3 - v1;
double t0[3] = -(a03*e1 + a01*e3);  // transport vector, tri 1
double t1[3] = -(a04*e2 + a02*e4);  // transport vector, tri 2
double sqr = dot(e0, e0);           // |edge|^2
double cos_theta = -dot(t0, t1) / sqr;
```

Geometric meaning of `cos_theta`: cosine of the angle between transported
normals — measures the rest dihedral angle. For a **flat** rest mesh:
`cos_theta = 1.0`. For a **curved** rest mesh: `cos_theta < 1.0`.

**Step 6: 4x4 stiffness matrix (outer product):**
```c
for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
        bending[4*i + j] += c[i] * c[j] * cos_theta * stiffness;
```

Stored at `flex_bending[17*e + 4*i + j]` for `i,j in 0..4`.

**Step 7: Curved reference coefficient (Garg correction):**
```c
double n[3];
mjuu_crossvec(n, e0, e1);  // normal to tri 1 (unnormalized)
bending[16] = dot(n, e2) * (a01 - a03) * (a04 - a02) * stiffness / (sqr * sqrt(sqr));
```

Stored at `flex_bending[17*e + 16]`.

For a **flat** mesh: `dot(n, e2) = 0` (e2 lies in the plane), so
`bending[16] = 0`. For a **curved** mesh: `dot(n, e2) != 0`, encoding
the out-of-plane offset. The curved reference term only activates when
both: (1) the mesh is non-flat, AND (2) the triangles have different
cotangent distributions at the edge endpoints.

### Build-time: precomputation gate

**Source:** `user_mesh.cc:4374` (MuJoCo 3.5.0)

```c
if (dim == 2 && (elastic2d == 1 || elastic2d == 3)) {
    bending.assign(nedge*17, 0);
    for (unsigned int e = 0; e < nedge; e++) {
        ComputeBending<StencilFlap>(...);
    }
}
```

- `elastic2d == 0`: membrane only (no bending)
- `elastic2d == 1`: bending only
- `elastic2d == 2`: nonlinear FEM only
- `elastic2d == 3`: membrane + bending

CortenForge computes bending for `dim == 2` with `thickness > 0` and
`young > 0` — matching MuJoCo's default shell plugin (elastic2d = 1 or 3).

### Runtime: bending force application

**Source:** `engine_passive.c:192–268` (MuJoCo 3.5.0)

**Gate conditions:**
1. `dim == 1 || flex_rigid[f]` → skip entire flex (`engine_passive.c:201`)
2. `dim == 2` → enter bending section (`engine_passive.c:205`)
3. `v[3] == -1` → skip boundary edge (`engine_passive.c:215`)
4. `has_spring` / `has_damping` → gate individual force components

**Diamond stencil assembly (runtime):**
```c
int v[4];
v[0] = m->flexedge_vert[2*e];      // edge endpoint 0
v[1] = m->flexedge_vert[2*e+1];    // edge endpoint 1
v[2] = m->flexedge_flap[2*e];      // opposite vertex, tri 1
v[3] = m->flexedge_flap[2*e+1];    // opposite vertex, tri 2 (-1 = boundary)
```

**Edge vectors (runtime, from current deformed positions):**
```c
mji_sub3(ed[0], xpos + 3*v[1], xpos + 3*v[0]);  // v1-v0
mji_sub3(ed[1], xpos + 3*v[2], xpos + 3*v[0]);  // v2-v0
mji_sub3(ed[2], xpos + 3*v[3], xpos + 3*v[0]);  // v3-v0
```

**Curved reference forces (cross products):**
```c
frc[1] = ed[1] x ed[2];   // (v2-v0) x (v3-v0)
frc[2] = ed[2] x ed[0];   // (v3-v0) x (v1-v0)
frc[3] = ed[0] x ed[1];   // (v1-v0) x (v2-v0)
frc[0] = -(frc[1] + frc[2] + frc[3]);
```

**Force accumulation (triple-nested loop):**
```c
for (int i = 0; i < 4; i++)
    for (int x = 0; x < 3; x++) {
        for (int j = 0; j < 4; j++) {
            spring[3*i+x] += b[17*e+4*i+j] * xpos[3*v[j]+x];  // B * pos
            damper[3*i+x] += b[17*e+4*i+j] * vel[j][x];        // B * vel
        }
        spring[3*i+x] += b[17*e+16] * frc[i][x];               // curved ref
    }
```

**Velocity source:** `vel[i]` reads from `d->qvel + body_dofadr[bodyid[v[i]]]`
for 3 DOFs (free vertex bodies have `body_dofnum == 3`). Pinned vertices
(no DOFs) contribute zero velocity.

**Global force insertion:**
```c
for (int i = 0; i < 4; i++) {
    int bodyid = m->flex_vertbodyid[v[i]];
    int adr = m->body_dofadr[bodyid];
    int dofnum = m->body_dofnum[bodyid];
    for (int x = 0; x < dofnum; x++) {
        d->qfrc_spring[adr+x] -= spring[3*i+x];
        d->qfrc_damper[adr+x] -= damper[3*i+x] * m->flex_damping[f];
    }
}
```

Key observations:
- Spring forces are **subtracted** from `qfrc_spring` (negative sign)
- Damper forces are **subtracted** AND **multiplied by `flex_damping[f]`**
- The bending stiffness is already baked into `b[]` coefficients — no
  runtime multiplication by a separate stiffness scalar
- `flex_damping[f]` is the proportional damping coefficient (separate from
  the B matrix which encodes bending stiffness)
- No stability clamp: the cotangent matrix is constant (precomputed), so the
  force is linear in position — no nonlinear blow-up risk
- Pinned vertices (`invmass == 0`): MuJoCo still computes their forces
  (B matrix includes them) but they have no DOFs, so `body_dofnum == 0`
  and the force insertion loop body is skipped

### Degenerate cases

| Case | MuJoCo behavior |
|------|----------------|
| Boundary edge (`flap[1] == -1`) | Skipped at runtime (`engine_passive.c:215`). `flex_bending` coefficients are all zero (not computed by `ComputeBending`). |
| Single triangle (no interior edges) | All edges are boundary. No bending forces. |
| `dim == 1` (cable) | Skipped by gate (`engine_passive.c:201`). No bending. |
| `dim == 3` (solid) | Uses FEM stiffness, not cotangent bending. Skipped by `dim == 2` gate. |
| `flex_rigid[f] == true` | Skipped by gate (`engine_passive.c:201`). |
| Zero thickness | `stiffness = 0` (thickness^3 = 0). All B matrix entries are zero. Forces are zero. |
| Degenerate triangle (zero area) | `volume = 0` → `stiffness = 3*mu*t^3 / (24*0) = Inf/NaN`. MuJoCo does not guard this — the model is assumed valid. CortenForge should guard (skip edge if `volume < epsilon`). |
| Flat rest mesh | `cos_theta = 1.0`, `b[16] = 0.0`. Pure cotangent Laplacian (no curved correction). |
| Curved rest mesh | `cos_theta < 1.0`, `b[16] != 0.0`. Garg correction active. |

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| Bending model | Cotangent Laplacian (Wardetzky/Garg). 4x4 B matrix precomputed per edge from rest geometry. Runtime: linear matrix-vector. | Bridson dihedral angle springs. Nonlinear atan2 each step. Per-vertex stability clamp. |
| Precomputed data | `flex_bending[17*e]`: 4x4 matrix + curved ref. `flexedge_flap[2*e]`: opposite vertices. | `flexhinge_vert[4*h]`: [ve0,ve1,va,vb]. `flexhinge_angle0[h]`: rest angle. No B matrix. |
| Force sign | Spring **subtracted** from `qfrc_spring`. Damper **subtracted** × `flex_damping[f]` from `qfrc_damper`. | Spring added as `grad * fm` to `qfrc_spring`. Damper added as `grad * fm` to `qfrc_damper`. Sign via angle_error sign. |
| Stability clamp | None needed (linear force, constant matrix). | Per-vertex clamp: `fm_max = 1/(dt^2 * |grad| * invmass)`. |
| Curved reference | `b[16]` encodes rest curvature via Garg correction. | No curved reference. Rest angle stored as scalar. |
| Bending stiffness source | Baked into B matrix via `mu * thickness^3 / volume`. | Kirchhoff-Love: `E*t^3 / (12*(1-nu^2))`. Different formula. |
| Topology | Per-edge flap stencil (edges, not hinges). | Per-hinge (adjacent triangle pairs, separate from edges). |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| Boundary flap vertex | `int -1` sentinel. `CreateFlapStencil()` sets `flap[0]` to the opposite vertex in the single adjacent triangle; only `flap[1]` stays `-1`. | `i32 -1` for both `flap[0]` and `flap[1]` on boundary edges (we set `[-1, -1]`). | Use `i32 -1` for boundary; check `flap[1] == -1` to skip boundary edges. **Divergence:** MuJoCo populates `flap[0]` for boundary edges; we don't. This is harmless — boundary edges are skipped at runtime (`flap[1]==-1` guard) and their `flex_bending` coefficients are zero, so `flap[0]` is never read. |
| Vertex DOF access | `body_dofadr[bodyid[v[i]]]` with `body_dofnum` DOFs | `flexvert_dofadr[v[i]]` with 3 DOFs (free vertex) | Substitute `flexvert_dofadr[v[i]]` for `body_dofadr[bodyid[v[i]]]`. Skip if `flexvert_dofadr[v[i]] == usize::MAX` (pinned). |
| `flex_bending` flat indexing | `17*e + 4*i + j` (C array) | `Vec<f64>` with same flat layout: `flex_bending[17*e + 4*i + j]` | Direct port — same indexing |
| `flexedge_flap` indexing | `flex_edgeflap[2*e + k]` (C flat array of ints) | `flexedge_flap: Vec<[i32; 2]>` indexed as `flexedge_flap[e][k]` | Use Rust array indexing instead of flat offset |
| Damping multiplier | `flex_damping[f]` | `model.flex_damping[flex_id]` | Direct port — same field name |
| Force accumulation | `qfrc_spring`, `qfrc_damper` (S4.7-prereq) | `data.qfrc_spring`, `data.qfrc_damper` | Direct port — same convention established in Phase 5 |
| Disable flags | `has_spring` / `has_damping` (C booleans) | `has_spring` / `has_damper` (already in `mj_fwd_passive`) | Use existing `has_spring` / `has_damper` locals |
| Rigid skip | `flex_rigid[f]` gate | `model.flex_rigid[flex_id]` (from T1) | Direct port — use T1's pre-computed flag |
| `flexedge_vert` | `flexedge_vert[2*e]`, `flexedge_vert[2*e+1]` | `model.flexedge_vert[e][0]`, `model.flexedge_vert[e][1]` | Use Rust array indexing |

### Conformant vs Extension Boundary

| Aspect | MuJoCo behavior | This spec | Conformance |
|--------|----------------|-----------|-------------|
| Cotangent precomputation (S1–S5) | `ComputeBending()` in `user_mesh.cc` | Same algorithm, same coefficients | **IDENTICAL** — conformant |
| Runtime force application (S6) | `engine_passive.c:206–268` | Same triple-nested loop, same sign, same damping | **IDENTICAL** — conformant |
| `FlexBendingModel` trait (S7) | No trait boundary | Trait with `CotangentBending` + `BridsonBending` impls | **EXTENSION** — trait does not alter cotangent output |
| `bending_model` MJCF attribute (S7) | No such attribute | Extension attribute; default = `Cotangent` (conformant) | **EXTENSION** — absent attribute gives conformant default |
| `BridsonBending` impl (S8) | No Bridson option | Preserves current dihedral code behind trait | **EXTENSION** — opt-in only |

---

## Architecture Decisions

### AD-1: Trait dispatch mechanism

**Problem:** Spec B introduces `FlexBendingModel` — the first trait in the
sim pipeline. The dispatch mechanism sets the pattern for post-v1.0 trait
architecture.

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | Dynamic dispatch (`Box<dyn FlexBendingModel>`) | Extensible, familiar | Heap allocation per flex, virtual call overhead in hot loop, doesn't match MuJoCo's static model |
| 2 | Enum dispatch (`match` on `FlexBendingType`) | Zero overhead, inline-able, matches MuJoCo's single-model architecture | Not extensible without modifying the enum |
| 3 | Compile-time generics (`fn<B: FlexBendingModel>`) | Zero overhead, extensible | Viral generics, complex API surface |

**Chosen:** Option 2 — enum dispatch. The bending model is known at model
build time and does not change per-step. A `match` on `FlexBendingType` in
`mj_fwd_passive()` gives zero-overhead dispatch with full inlining. This
matches MuJoCo's architecture (no runtime model selection) and keeps the
API surface simple. Post-v1.0, if more models are needed, enum variants are
added.

### AD-2: Cotangent and Bridson coexistence

**Problem:** The cotangent model uses per-edge topology (`flexedge_flap`,
`flex_bending`). The Bridson model uses per-hinge topology (`flexhinge_vert`,
`flexhinge_angle0`). Both must coexist.

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | Unified topology — convert Bridson to use per-edge flap stencil | Single topology set | Requires rewriting Bridson internals, changes its semantics |
| 2 | Dual topology — keep both `flexhinge_*` and `flexedge_flap` | No Bridson changes, clean separation | Slightly more Model storage |

**Chosen:** Option 2 — dual topology. The hinge topology (`flexhinge_*`) is
correct and tested for Bridson. The edge flap topology (`flexedge_flap`) is
needed for cotangent. Both are computed at build time with negligible cost.
Bridson code is moved into `BridsonBending::apply()` with zero algorithm
changes. The hinge topology persists regardless of `bending_model` choice
(negligible storage cost; simpler than conditional computation).

---

## Specification

### S1. `flexedge_flap` topology computation

**File:** `sim/L0/mjcf/src/builder/flex.rs` (after edge extraction, ~line 270)
**MuJoCo equivalent:** `CreateFlapStencil()` in `user_mesh.cc:3593–3639`
**Design decision:** Uses the existing `edge_elements` HashMap (lines 272–285)
which already maps canonical edge keys `(min, max)` to element indices. For
each edge with exactly 2 adjacent elements, finds the opposite vertex in
each triangle. Boundary edges (1 adjacent element) get `flap = [-1, -1]`. MuJoCo's
`CreateFlapStencil()` actually populates `flap[0]` with the opposite vertex
from the single adjacent triangle, leaving only `flap[1] = -1`. We set both
to `-1` for simplicity — this is harmless because boundary edges are skipped
at runtime (`flap[1] == -1` guard) and their `flex_bending` coefficients are
all zero (see Convention Notes for full divergence documentation).

**Implementation note:** The flap array must be indexed by edge index
(0..nflexedge), not by the edge's position in the HashMap. The
`flexedge_flap` entry for edge `e` corresponds to the same edge as
`flexedge_vert[e]`. Since edges are pushed in HashMap iteration order (which
is non-deterministic), we need to either: (a) build a lookup from canonical
edge key to edge index during edge extraction, or (b) compute flaps during
edge extraction in the same loop. Option (b) is simpler — compute flap
vertices during the existing edge loop where `edge_elements` is available.

However, `flexedge_flap` must be populated for ALL edges (including those
not adjacent to 2 elements), so it should be initialized to `[-1, -1]` and
filled for interior edges.

**After (new code added to `process_flex_bodies`):**

During the edge extraction loop (lines 245–269), we track a mapping from
canonical edge key to global edge index. Then after the `edge_elements` map
is built, we iterate it to fill `flexedge_flap`:

```rust
// During edge extraction (modify existing loop):
let mut edge_key_to_global: HashMap<(usize, usize), usize> = HashMap::new();
// ... existing edge push code ...
// After pushing edge [vert_start + a, vert_start + b]:
edge_key_to_global.insert((a, b), self.nflexedge - 1);

// After edge extraction is complete:
// Initialize flexedge_flap for all new edges to [-1, -1]
for _ in 0..edge_set.len() {
    self.flexedge_flap.push([-1, -1]);
}

// Fill opposite vertices for interior edges
for ((ve0_local, ve1_local), elems) in &edge_elements {
    let edge_idx = edge_key_to_global[&(*ve0_local, *ve1_local)];
    let flap_local_idx = edge_idx - edge_start; // index into flex-local range
    if elems.len() >= 2 {
        // Find opposite vertex in each element
        let opp_0 = flex.elements[elems[0]]
            .iter()
            .find(|&&v| v != *ve0_local && v != *ve1_local)
            .copied()
            .unwrap();
        let opp_1 = flex.elements[elems[1]]
            .iter()
            .find(|&&v| v != *ve0_local && v != *ve1_local)
            .copied()
            .unwrap();
        self.flexedge_flap[edge_idx] = [
            (vert_start + opp_0) as i32,
            (vert_start + opp_1) as i32,
        ];
    }
    // else: boundary edge — already [-1, -1]
}
```

**Model field:**

```rust
/// Per-edge flap vertices: opposite vertices in adjacent triangles forming
/// the diamond stencil. `flexedge_flap[e][0]` = opposite vertex in tri 1,
/// `flexedge_flap[e][1]` = opposite vertex in tri 2 (-1 for boundary edges).
/// Length `nflexedge`.
pub flexedge_flap: Vec<[i32; 2]>,
```

---

### S2. Cotangent weight precomputation

**File:** `sim/L0/mjcf/src/builder/flex.rs` (new function `compute_bending_coefficients`)
**MuJoCo equivalent:** `ComputeBending()` in `user_mesh.cc:3666–3712`,
`cot()` at lines 3645–3654, `ComputeVolume()` at lines 3655–3663
**Design decision:** A standalone function `compute_bending_coefficients()`
takes the rest vertex positions, edge vertices, and flap vertices, and
returns a `[f64; 17]` array for one edge. This matches MuJoCo's
`ComputeBending()` which operates per-edge. The function is called during
`process_flex_bodies()` for each interior edge of a `dim == 2` flex with
`young > 0` and `thickness > 0`.

**Helper: `cot_angle()`:**

```rust
/// Cotangent of angle at v0 in triangle (v0, v1, v2).
/// Returns dot(e1,e2) / |cross(e1,e2)| = cos(alpha) / sin(alpha).
fn cot_angle(v0: Vector3<f64>, v1: Vector3<f64>, v2: Vector3<f64>) -> f64 {
    let e1 = v1 - v0;
    let e2 = v2 - v0;
    let cross = e1.cross(&e2);
    let cross_norm = cross.norm();
    if cross_norm < 1e-30 {
        return 0.0; // degenerate triangle guard
    }
    e1.dot(&e2) / cross_norm
}
```

**Helper: `triangle_area()`:**

```rust
/// Area of triangle (v0, v1, v2) = |cross(v1-v0, v2-v0)| / 2.
fn triangle_area(v0: Vector3<f64>, v1: Vector3<f64>, v2: Vector3<f64>) -> f64 {
    let e1 = v1 - v0;
    let e2 = v2 - v0;
    e1.cross(&e2).norm() / 2.0
}
```

**Main computation (per edge):**

```rust
/// Compute 17 bending coefficients for one edge per MuJoCo's ComputeBending().
///
/// v[0], v[1]: edge endpoints (rest positions)
/// v[2]: opposite vertex in triangle 1
/// v[3]: opposite vertex in triangle 2
/// mu: shear modulus = young / (2 * (1 + poisson))
/// thickness: shell thickness
///
/// Returns [f64; 17]: b[0..16] = 4x4 stiffness matrix, b[16] = Garg curved ref.
fn compute_bending_coefficients(
    v: [Vector3<f64>; 4],
    mu: f64,
    thickness: f64,
) -> [f64; 17] {
    let mut b = [0.0f64; 17];

    // Step 1: cotangent weights
    let a01 = cot_angle(v[0], v[1], v[2]); // angle at v[0] in tri 1
    let a02 = cot_angle(v[0], v[3], v[1]); // angle at v[0] in tri 2
    let a03 = cot_angle(v[1], v[2], v[0]); // angle at v[1] in tri 1
    let a04 = cot_angle(v[1], v[0], v[3]); // angle at v[1] in tri 2

    // Step 2: weight vector
    let c = [
        a03 + a04,        // sum of cots at v[1]
        a01 + a02,        // sum of cots at v[0]
        -(a01 + a03),     // neg sum from tri 1
        -(a02 + a04),     // neg sum from tri 2
    ];

    // Step 3: diamond volume (sum of triangle areas)
    let vol1 = triangle_area(v[0], v[1], v[2]);
    let vol2 = triangle_area(v[1], v[0], v[3]);
    let volume = vol1 + vol2;
    if volume < 1e-30 {
        return b; // degenerate diamond: all zero coefficients
    }

    // Step 4: material stiffness
    let stiffness = 3.0 * mu * thickness.powi(3) / (24.0 * volume);

    // Step 5: transport vectors and cos_theta
    let e0 = v[1] - v[0]; // shared edge
    let e1 = v[2] - v[0]; // to opp vertex in tri 1
    let e2 = v[3] - v[0]; // to opp vertex in tri 2
    let e3 = v[2] - v[1];
    let e4 = v[3] - v[1];
    // Transport vectors (parallel transport of edge normals within each tri)
    let t0 = -(e1 * a03 + e3 * a01); // tri 1
    let t1 = -(e2 * a04 + e4 * a02); // tri 2
    let sqr = e0.dot(&e0); // |edge|^2
    let cos_theta = if sqr < 1e-30 {
        1.0 // degenerate edge: treat as flat
    } else {
        -t0.dot(&t1) / sqr
    };

    // Step 6: 4x4 stiffness matrix (outer product)
    for i in 0..4 {
        for j in 0..4 {
            b[4 * i + j] = c[i] * c[j] * cos_theta * stiffness;
        }
    }

    // Step 7: curved reference coefficient (Garg correction)
    let n = e0.cross(&e1); // normal to tri 1 (unnormalized)
    if sqr > 1e-30 {
        b[16] = n.dot(&e2) * (a01 - a03) * (a04 - a02) * stiffness
            / (sqr * sqr.sqrt());
    }

    b
}
```

---

### S3. `flex_bending` storage and population

**File:** `sim/L0/core/src/types/model.rs` (flex per-edge arrays section, ~line 450)
**MuJoCo equivalent:** `m->flex_bending` in `mjModel`, shaped `(nflexedge, 17)`
**Design decision:** Store as flat `Vec<f64>` with `17 * nflexedge` entries,
matching MuJoCo's layout. Indexed as `flex_bending[17 * e + 4 * i + j]` for
the 4x4 matrix and `flex_bending[17 * e + 16]` for the Garg coefficient.
All entries initialized to zero; only interior edges of `dim == 2` flexes
with valid material get non-zero coefficients.

**Model field:**

```rust
/// Per-edge cotangent bending coefficients (Wardetzky/Garg cotangent Laplacian).
/// Layout: 17 f64 per edge, flat — `flex_bending[17*e + 4*i + j]` for the
/// 4x4 stiffness matrix, `flex_bending[17*e + 16]` for the Garg curved
/// reference coefficient. Zero for boundary edges and non-dim-2 flexes.
/// Length `17 * nflexedge`.
pub flex_bending: Vec<f64>,
```

**Population in `process_flex_bodies()`:**

After `flexedge_flap` is populated (S1), and after per-flex material
properties are available:

```rust
// Compute cotangent bending coefficients for dim==2 flex with valid material
if flex.dim == 2 && flex.young > 0.0 && flex.thickness > 0.0 {
    let mu = flex.young / (2.0 * (1.0 + flex.poisson));
    for local_e in 0..edge_set.len() {
        let global_e = edge_start + local_e;
        let flap = self.flexedge_flap[global_e];
        if flap[1] == -1 {
            continue; // boundary edge: coefficients stay zero
        }
        let verts = [
            flex.vertices[self.flexedge_vert[global_e][0] - vert_start],
            flex.vertices[self.flexedge_vert[global_e][1] - vert_start],
            flex.vertices[(flap[0] as usize) - vert_start],
            flex.vertices[(flap[1] as usize) - vert_start],
        ];
        let coeffs = compute_bending_coefficients(verts, mu, flex.thickness);
        let base = 17 * global_e;
        self.flex_bending[base..base + 17].copy_from_slice(&coeffs);
    }
}
```

**Pre-allocation:** Push 17 zeros per edge during edge extraction (in the
existing edge push loop, immediately after pushing to `flexedge_vert`).
This keeps `flex_bending` aligned with `flexedge_vert` — edge `e` always
has its coefficients at `flex_bending[17*e..17*e+17]`. The cotangent
computation (above) then overwrites the zeros for interior edges only.

---

### S4. `FlexBendingType` enum and MJCF parsing

**File:** `sim/L0/mjcf/src/types.rs` (near `MjcfFlex` struct, ~line 3860)
**File:** `sim/L0/mjcf/src/parser.rs` (flex parsing section)
**MuJoCo equivalent:** None — CortenForge extension
**Design decision:** Enum dispatch (AD-1). Default `Cotangent` for MuJoCo
conformance. Attribute `bending_model` parsed from `<flex><elasticity>`.
Models without the attribute get `Cotangent` (conformant default).

**Type definition:**

```rust
/// Bending model for dim=2 flex bodies.
/// Cotangent (default) matches MuJoCo's cotangent Laplacian.
/// Bridson preserves the dihedral angle spring model (CortenForge extension).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FlexBendingType {
    /// Wardetzky/Garg cotangent Laplacian (MuJoCo-conformant, default).
    #[default]
    Cotangent,
    /// Bridson dihedral angle springs (nonlinear, large-deformation accurate).
    Bridson,
}
```

**MjcfFlex field:**

```rust
/// Bending model selection (default: Cotangent = MuJoCo-conformant).
/// CortenForge extension attribute — not in MuJoCo schema.
pub bending_model: FlexBendingType,
```

**Parser:** Parse `bending_model` from `<elasticity>` child element of
`<flex>`:

```rust
// In parse_elasticity() or equivalent:
let bending_model = match attrs.get("bending_model").map(|s| s.as_str()) {
    Some("cotangent") | None => FlexBendingType::Cotangent,
    Some("bridson") => FlexBendingType::Bridson,
    Some(other) => return Err(/* unknown bending_model value */),
};
```

**Model field:**

```rust
/// Per-flex: bending model type (Cotangent or Bridson). Default: Cotangent.
/// Length `nflex`.
pub flex_bending_type: Vec<FlexBendingType>,
```

Pushed during `process_flex_bodies()`:
```rust
self.flex_bending_type.push(flex.bending_model);
```

---

### S5. (Reserved — absorbed into S2/S3)

**Section numbering note:** The session plan listed 5 build-time sections
(S1: topology, S2: cotangent weights, S3: material stiffness, S4: 4x4
matrix, S5: curved reference). This spec reorganized them because cotangent
weights, material stiffness, the 4x4 matrix, and the curved reference
coefficient are all computed in a single per-edge function
(`compute_bending_coefficients()`) — splitting them into 4 sections would
create artificial boundaries. The mapping:

| Session plan | Spec section | Rationale |
|-------------|-------------|-----------|
| S1: topology | S1 | Direct match |
| S2: cotangent weights | S2 (Steps 1–2) | Part of `compute_bending_coefficients()` |
| S3: material stiffness | S2 (Steps 3–4) | Part of same function |
| S4: 4x4 matrix | S2 (Steps 5–6) | Part of same function |
| S5: curved reference | S2 (Step 7) | Part of same function |
| *(new)* | S3: storage + population | Separated from algorithm for clarity |
| *(new)* | S4: enum + MJCF parsing | Independent of S2 |

Session 10's prompt says "Implement S1 through S5 ONLY." With this
reorganization, Session 10 implements S1 through S4 (covering all 5 build-time
tasks). The implementer should follow the spec's section numbering, not the
session plan's.

---

### S6. Runtime cotangent force application

**File:** `sim/L0/core/src/forward/passive.rs` (bending section, lines ~548–670)
**MuJoCo equivalent:** `engine_passive.c:192–268`
**Design decision:** Replaces the Bridson bending loop with a
`FlexBendingType`-dispatched section. For `Cotangent`, implements the
MuJoCo triple-nested loop. For `Bridson`, delegates to the moved Bridson
code (S8). The cotangent path uses `flexedge_flap` and `flex_bending` to
apply forces directly to vertex DOFs — no edge Jacobian needed (forces are
per-vertex, not per-edge).

**Cotangent runtime implementation:**

```rust
// Replace existing bending hinge loop (lines 548-670) with:
// Bending forces — dispatched by FlexBendingType
for f in 0..model.nflex {
    let dim = model.flex_dim[f];
    if dim != 2 || model.flex_rigid[f] {
        continue;
    }

    match model.flex_bending_type[f] {
        FlexBendingType::Cotangent => {
            // Cotangent Laplacian bending (MuJoCo-conformant)
            let edge_adr = model.flex_edgeadr[f];
            let edge_num = model.flex_edgenum[f];
            let damping_coeff = model.flex_damping[f];

            for local_e in 0..edge_num {
                let e = edge_adr + local_e;
                let flap = model.flexedge_flap[e];
                if flap[1] == -1 {
                    continue; // boundary edge
                }

                // Diamond stencil vertices: [edge0, edge1, opp_tri1, opp_tri2]
                let v = [
                    model.flexedge_vert[e][0],
                    model.flexedge_vert[e][1],
                    flap[0] as usize,
                    flap[1] as usize,
                ];

                // Runtime edge vectors (from current deformed positions)
                let xpos = &data.flexvert_xpos;
                let ed0 = xpos[v[1]] - xpos[v[0]]; // v1 - v0
                let ed1 = xpos[v[2]] - xpos[v[0]]; // v2 - v0
                let ed2 = xpos[v[3]] - xpos[v[0]]; // v3 - v0

                // Curved reference forces (cross products)
                let frc1 = ed1.cross(&ed2); // (v2-v0) x (v3-v0)
                let frc2 = ed2.cross(&ed0); // (v3-v0) x (v1-v0)
                let frc3 = ed0.cross(&ed1); // (v1-v0) x (v2-v0)
                let frc0 = -(frc1 + frc2 + frc3);
                let frc = [frc0, frc1, frc2, frc3];

                // Read velocities for damper
                let read_vel = |vi: usize| -> Vector3<f64> {
                    let dof = model.flexvert_dofadr[vi];
                    if dof == usize::MAX {
                        Vector3::zeros()
                    } else {
                        Vector3::new(
                            data.qvel[dof],
                            data.qvel[dof + 1],
                            data.qvel[dof + 2],
                        )
                    }
                };
                let vel = [read_vel(v[0]), read_vel(v[1]), read_vel(v[2]), read_vel(v[3])];

                // B matrix coefficients for this edge
                let b_base = 17 * e;

                // Force accumulation: spring[3*i+x] and damper[3*i+x]
                let mut spring = [0.0f64; 12]; // 4 vertices x 3 axes
                let mut damper = [0.0f64; 12];

                for i in 0..4 {
                    for x in 0..3 {
                        for j in 0..4 {
                            let b_ij = model.flex_bending[b_base + 4 * i + j];
                            spring[3 * i + x] += b_ij * xpos[v[j]][x];
                            damper[3 * i + x] += b_ij * vel[j][x];
                        }
                        // Curved reference contribution
                        spring[3 * i + x] +=
                            model.flex_bending[b_base + 16] * frc[i][x];
                    }
                }

                // Insert forces into qfrc_spring and qfrc_damper
                for i in 0..4 {
                    let dof = model.flexvert_dofadr[v[i]];
                    if dof == usize::MAX {
                        continue; // pinned vertex: no DOFs
                    }
                    for x in 0..3 {
                        if has_spring {
                            // Spring subtracted (negative sign matches MuJoCo)
                            data.qfrc_spring[dof + x] -= spring[3 * i + x];
                        }
                        if has_damper {
                            // Damper subtracted and multiplied by flex_damping
                            data.qfrc_damper[dof + x] -=
                                damper[3 * i + x] * damping_coeff;
                        }
                    }
                }
            }
        }
        FlexBendingType::Bridson => {
            // Delegate to Bridson dihedral angle springs (S8)
            apply_bridson_bending(model, data, f, has_spring, has_damper);
        }
    }
}
```

**Critical correctness points:**
1. Spring forces **subtracted** (`-=`) — matches MuJoCo sign convention
2. Damper forces **subtracted** AND **multiplied by `flex_damping[f]`**
3. No stability clamp — cotangent matrix is constant, linear force
4. Pinned vertices: `dofadr == usize::MAX` → skip force insertion (DOF count 0)
5. Boundary edges: `flap[1] == -1` → skip (zero coefficients anyway)
6. `xpos[v[j]][x]` accesses axis `x` of `Vector3` — Rust indexing via
   `v[0]`, `v[1]`, `v[2]` on `Vector3<f64>` (nalgebra's `Index<usize>`)

---

### S7. `FlexBendingModel` trait definition and `CotangentBending`

**File:** `sim/L0/core/src/forward/passive.rs`
**MuJoCo equivalent:** None — CortenForge extension
**Design decision:** Per AD-1, use enum dispatch (not dynamic dispatch). The
"trait" is conceptual — implemented as a `match` on `FlexBendingType` in the
bending dispatch loop (S6 above). No Rust `trait` keyword needed. The `match`
arms inline the full algorithm for each model.

The enum dispatch in S6 IS the trait architecture. Cotangent is the `match`
arm body in S6. Bridson is the `apply_bridson_bending()` call. No separate
`trait FlexBendingModel` definition is needed — the `match` gives the same
zero-overhead dispatch.

**Why no `trait` keyword:** A Rust `trait` adds API surface (generic bounds,
associated types) without benefit when there are exactly two implementations,
both known at compile time, selected by a Model field. The `match` is
simpler, equally extensible (add enum variant + match arm), and the compiler
inlines both paths.

---

### S8. `BridsonBending` — preserve existing dihedral code

**File:** `sim/L0/core/src/forward/passive.rs` (new function
`apply_bridson_bending`)
**MuJoCo equivalent:** None — CortenForge extension
**Design decision:** Move existing bending code (lines 548–670) into
`apply_bridson_bending(model, data, flex_id, has_spring, has_damper)`.
Zero algorithm changes. The function iterates `flexhinge_*` arrays for the
given flex and applies dihedral angle spring/damper forces with the existing
per-vertex stability clamp.

**Function signature:**

```rust
/// Bridson dihedral angle bending (CortenForge extension).
/// Exact code from current passive.rs:548-670, scoped to one flex body.
fn apply_bridson_bending(
    model: &Model,
    data: &mut Data,
    flex_id: usize,
    has_spring: bool,
    has_damper: bool,
) {
    let dt = model.timestep;
    for h in 0..model.nflexhinge {
        if model.flexhinge_flexid[h] != flex_id {
            continue;
        }
        // ... exact current code from lines 553-670 ...
        // No algorithm changes. Only scoped to flex_id.
    }
}
```

The filtering `if model.flexhinge_flexid[h] != flex_id { continue; }` is
new (current code iterates ALL hinges). This is correct because the dispatch
loop in S6 iterates per-flex, so each hinge is visited exactly once.

**Performance note:** The filter loop is O(nflexhinge) per flex call. Since
the S6 dispatch loop iterates all `nflex` flex bodies, the total Bridson
work is O(nflex × nflexhinge) — vs O(nflexhinge) for the current flat loop.
For nflex=1 (common case), this is identical. For multiple flex bodies, the
extra comparisons (`flexhinge_flexid[h] != flex_id`) are negligible vs the
bending computation itself. If performance profiling shows this matters,
add `flex_hingeadr[f]` / `flex_hingenum[f]` address arrays for O(1) range
access (tracked as future optimization, not DT-gated).

---

## Acceptance Criteria

### AC1: Equilateral diamond precomputation *(runtime test — analytically derived)*
**Given:** Two equilateral triangles sharing edge (0,0,0)→(1,0,0), with
opposite vertices at (0.5, √3/2, 0) and (0.5, -√3/2, 0). Material:
`young=1e4`, `poisson=0.3`, `thickness=0.01`.
**After:** Model build
**Assert:** `flex_bending[17*e + ...]`:
- All 4 cotangent values = `1/√3 ≈ 0.57735`
- Weight vector `c = [2/√3, 2/√3, -2/√3, -2/√3]`
- Diamond area = `√3/2 ≈ 0.8660`
- `mu = 1e4/2.6 ≈ 3846.154`
- `stiffness = 3 × 3846.154 × 1e-6 / (24 × 0.8660) ≈ 5.551e-4`
- `cos_theta = 1.0` (flat)
- Each B matrix entry magnitude = `stiffness × 4/3 ≈ 7.402e-4`
- B matrix sign pattern: `[[+,+,-,-],[+,+,-,-],[-,-,+,+],[-,-,+,+]]`
- `b[16] = 0.0` (flat mesh)
- All row sums = 0 (cotangent Laplacian property)
**Field:** `model.flex_bending`
**Tolerance:** `1e-10` (exact arithmetic on known geometry)

### AC2: Boundary edge produces zero coefficients *(runtime test — analytically derived)*
**Given:** Single triangle (3 vertices, 1 element). All 3 edges are boundary.
**After:** Model build
**Assert:** `flex_bending` is all zeros for all edges. `flexedge_flap[e][1] == -1` for all edges.
**Field:** `model.flex_bending`, `model.flexedge_flap`

### AC3: Cotangent runtime force — flat diamond deflection *(runtime test — analytically derived)*
**Given:** Two equilateral triangles (same geometry as AC1), `dim=2`,
`young=1e4`, `poisson=0.3`, `thickness=0.01`, `density=1000`, no gravity.
All 4 vertices free (no pins). Perturb v[2] z-position by +0.01 (out of
plane) via `qpos`. No velocity perturbation. Call `forward()` once.
**After:** `forward()` (passive force computation only)
**Assert:**
- `qfrc_spring` z-component on v[2] ≈ `-7.402e-6` (restoring toward plane)
- `qfrc_spring` z-component on v[3] ≈ `-7.402e-6` (symmetric with v[2])
- `qfrc_spring` z-component on v[0] ≈ `+7.402e-6` (edge endpoint)
- `qfrc_spring` z-component on v[1] ≈ `+7.402e-6` (edge endpoint)
- Sum of z-forces across all 4 vertices = 0 (Laplacian property)
- x and y force components are zero (perturbation is z-only; at rest
  positions, B×xpos is zero in x/y due to Laplacian null space)

**Derivation:** Only v[2] has nonzero z-coordinate (0.01). `spring[3*i+2] =
b[4*i+2] * 0.01`. With `b[4*i+2] = c[i]*c[2]*stiffness`:
`c[2] = -2/√3`, so `b[0][2] = (2/√3)(-2/√3)*s = -(4/3)*s`,
`b[2][2] = (-2/√3)(-2/√3)*s = (4/3)*s`. Force insertion subtracts:
`qfrc_spring[v2_z] = -(4/3)*s*0.01 ≈ -7.402e-6`.
**Field:** `data.qfrc_spring`
**Tolerance:** `1e-10` for magnitudes, `1e-12` for force balance sum.

### AC4: Curved reference — non-flat rest mesh *(runtime test — analytically derived)*
**Given:** Asymmetric diamond with v[3] out of plane: v0=(0,0,0), v1=(2,0,0),
v2=(0.5,1,0), v3=(1.5,-0.8,0.5). `young=1e4`, `poisson=0.3`,
`thickness=0.01`.
**After:** Model build
**Assert:**
- `cos_theta ≈ 0.848` (curved mesh, derived from transport vectors on
  asymmetric geometry — EGT-2b)
- `flex_bending[17*e + 16] ≈ 3.3e-5` (order of magnitude from EGT-2b;
  exact value depends on area/stiffness computation for this geometry)
- All B matrix row sums = 0 (Laplacian property preserved for curved meshes)
**Field:** `model.flex_bending`
**Tolerance:** `1e-3` for `cos_theta`, `1e-6` for `b[16]` (hand-computed
values from EGT-2b are approximate; refine during implementation with exact
arithmetic)

### AC5: `dim == 1` flex produces zero bending *(runtime test — analytically derived)*
**Given:** Cable flex (`dim=1`), `young=1e4`.
**After:** `mj_step()`
**Assert:** No bending forces applied to any vertex DOFs from the bending
section. `qfrc_spring` from bending = 0 for all cable vertices.
**Field:** `data.qfrc_spring`

### AC6: `flex_rigid` skip *(runtime test — analytically derived)*
**Given:** `dim=2` flex with all vertices pinned (`flex_rigid[f] == true`).
**After:** `mj_step()`
**Assert:** Bending code path skipped (no forces applied). Verified by
checking no writes to `qfrc_spring` for this flex's edges.
**Field:** `data.qfrc_spring`

### AC7: Bridson regression — identical results with `bending_model="bridson"` *(runtime test)*
**Given:** The bending strip model from existing test `ac6_bending_stiffness`
(8 vertices, 6 triangles, 2 pinned), with `bending_model="bridson"` added
to `<elasticity>`.
**After:** 2000 steps
**Assert:** Vertex positions exactly match pre-Spec-B results (bit-identical
to current Bridson implementation). No NaN. Tip below root.
**Field:** `data.flexvert_xpos`
**Tolerance:** `1e-14` (bit-identical)

### AC8: Damper with `flex_damping` multiplier *(runtime test — analytically derived)*
**Given:** Two equilateral triangles, `damping=0.5`, `young=1e4`,
`poisson=0.3`, `thickness=0.01`. Perturb v[2] velocity in z.
**After:** 1 step (compute passive forces only, no integration)
**Assert:** `qfrc_damper` values are non-zero and are scaled by
`flex_damping[f] = 0.5`. Specifically: `qfrc_damper[dof] = -damper * 0.5`
where `damper` is the B matrix × velocity product.
**Field:** `data.qfrc_damper`

### AC9: DISABLE_SPRING gate *(runtime test — analytically derived)*
**Given:** Cotangent bending model with valid material. `disableflags` has
`DISABLE_SPRING` set.
**After:** 1 step
**Assert:** `qfrc_spring` is zero for all bending-related DOFs. `qfrc_damper`
is non-zero (damper not disabled).
**Field:** `data.qfrc_spring`, `data.qfrc_damper`

### AC10: DISABLE_DAMPER gate *(runtime test — analytically derived)*
**Given:** Cotangent bending model with valid material and velocity
perturbation. `disableflags` has `DISABLE_DAMPER` set.
**After:** 1 step
**Assert:** `qfrc_damper` is zero. `qfrc_spring` is non-zero.
**Field:** `data.qfrc_spring`, `data.qfrc_damper`

### AC11: No stability clamp needed *(runtime test — analytically derived)*
**Given:** Very high stiffness (`young=1e12`, `thickness=0.005`,
`timestep=0.01`), cotangent bending. Same strip geometry as ac20.
**After:** 500 steps
**Assert:** No NaN, no blow-up. Vertex positions remain finite. The
cotangent model is stable without the Bridson-style `fm_max` clamp because
forces are linear in position (constant B matrix).
**Field:** `data.flexvert_xpos`

### AC12: `flexedge_flap` topology correctness *(runtime test — analytically derived)*
**Given:** 4x4 vertex grid (16 vertices, 18 triangles, 33 edges).
**After:** Model build
**Assert:** Interior edges have `flap[0] >= 0` and `flap[1] >= 0`. Boundary
edges have `flap[1] == -1`. Interior edge count matches expected (edges
shared by exactly 2 triangles). For each interior edge, the 4 vertices
(edge + flap) form a valid diamond: all 4 are distinct, and the 2 flap
vertices are NOT on the edge.
**Field:** `model.flexedge_flap`

### AC13: Zero thickness produces zero bending *(runtime test — analytically derived)*
**Given:** `dim=2` flex with `thickness=0.0`, `young=1e4`.
**After:** Model build
**Assert:** `flex_bending` is all zeros (stiffness formula has `thickness^3 = 0`).
**Field:** `model.flex_bending`

### AC14: `FlexBendingType` default is `Cotangent` *(code review)*
**Given:** MJCF model with no `bending_model` attribute on `<elasticity>`.
**Assert:** `model.flex_bending_type[f] == FlexBendingType::Cotangent`.

### AC15: Mixed bending models — two flex bodies *(runtime test)*
**Given:** Two flex bodies in one model: flex A with `bending_model="cotangent"`,
flex B with `bending_model="bridson"`.
**After:** 100 steps
**Assert:** Both simulate without error. Flex A uses cotangent forces
(no stability clamp). Flex B uses Bridson forces (with stability clamp).
No cross-contamination.
**Field:** `data.flexvert_xpos`

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (equilateral precomp) | T1 | Direct |
| AC2 (boundary edge) | T2 | Direct |
| AC3 (cotangent runtime force) | T3 | Direct |
| AC4 (curved reference) | T4 | Direct |
| AC5 (dim=1 skip) | T5 | Direct |
| AC6 (flex_rigid skip) | T6 | Direct |
| AC7 (Bridson regression) | T7 | Direct |
| AC8 (damper multiplier) | T8 | Direct |
| AC9 (DISABLE_SPRING) | T9 | Direct |
| AC10 (DISABLE_DAMPER) | T10 | Direct |
| AC11 (no stability clamp) | T11 | Direct |
| AC12 (flap topology) | T12 | Direct |
| AC13 (zero thickness) | T13 | Edge case |
| AC14 (default bending type) | T14 | Direct (code review + runtime) |
| AC15 (mixed models) | T15 | Integration |

---

## Test Plan

### T1: Equilateral diamond precomputation → AC1
**Model:** 4 vertices, 2 triangles, 1 interior edge. v0=(0,0,0), v1=(1,0,0),
v2=(0.5,√3/2,0), v3=(0.5,-√3/2,0). `dim=2`, `young=1e4`, `poisson=0.3`,
`thickness=0.01`.
**Expected:** `flex_bending` coefficients match EGT-2:
- `stiffness ≈ 5.551e-4` (tolerance 1e-10)
- B matrix entries `|b[i][j]| ≈ 7.402e-4` with sign pattern per EGT-2
- `b[16] = 0.0` (flat mesh)
- All row sums = 0
**Source:** Analytically derived from known equilateral geometry.

### T2: Boundary edge — single triangle → AC2
**Model:** 3 vertices, 1 triangle, 3 edges (all boundary).
**Expected:** All `flexedge_flap[e][1] == -1`. All `flex_bending` zeros.
**Source:** Analytically derived.

### T3: Cotangent runtime force — deflected diamond → AC3
**Model:** Same equilateral diamond as T1, all vertices free (no pins),
no gravity. Perturb v[2] z-position by +0.01 via qpos. No velocity. Call
`forward()` once (no integration).
**Expected:** After forward(), `qfrc_spring` z-components:
- v[0]: `+7.402e-6 ± 1e-10`
- v[1]: `+7.402e-6 ± 1e-10`
- v[2]: `-7.402e-6 ± 1e-10`
- v[3]: `-7.402e-6 ± 1e-10`
- Sum = 0 ± 1e-12
- x, y components: 0 ± 1e-14
**Source:** Analytically derived: `qfrc_spring[vi_z] = -b[4*i+2] * 0.01` where
`b[4*i+2] = c[i]*c[2]*stiffness` and `stiffness ≈ 5.551e-4`.

### T4: Curved reference — asymmetric out-of-plane diamond → AC4
**Model:** v0=(0,0,0), v1=(2,0,0), v2=(0.5,1,0), v3=(1.5,-0.8,0.5).
`young=1e4`, `poisson=0.3`, `thickness=0.01`.
**Expected:**
- `cos_theta ≈ 0.848 ± 1e-3` (from EGT-2b transport vector computation)
- `flex_bending[17*e + 16] ≈ 3.3e-5 ± 1e-6` (from EGT-2b Garg formula)
- All B matrix row sums = 0 ± 1e-12 (Laplacian property)
**Source:** Analytically derived from EGT-2b asymmetric geometry. Values are
approximate from rubric EGT-2b; refine with exact arithmetic during
implementation (compute cos_theta and b[16] from the four vertex positions
using the formulas in S2 and verify the test matches).

### T5: dim=1 cable — no bending → AC5
**Model:** `dim=1` cable flex, 4 vertices, 3 edges. `young=1e4`.
**Expected:** After step, no bending forces. `qfrc_spring` only from edge
spring-damper, not bending.
**Source:** Analytically derived (dim gate).

### T6: Rigid flex — bending skipped → AC6
**Model:** `dim=2` flex with all vertices pinned. `young=1e4`.
**Expected:** `flex_rigid[f] == true`. Bending loop skipped.
**Source:** Analytically derived (rigid gate).

### T7: Bridson regression → AC7
**Model:** Existing bending strip model (`ac6_bending_stiffness` geometry)
with `bending_model="bridson"` added.
**Expected:** After 2000 steps, vertex positions match current behavior
within `1e-14`. No NaN. Tip below root.
**Source:** Regression against pre-Spec-B behavior. Run current code to
capture expected values before implementing.

### T8: Damper with flex_damping multiplier → AC8
**Model:** Equilateral diamond, `damping=0.5`, `young=1e4`, `poisson=0.3`,
`thickness=0.01`. Perturb v[2] z-velocity.
**Expected:** `qfrc_damper` values scaled by 0.5 relative to the raw
B-matrix × velocity product.
**Source:** Analytically derived.

### T9: DISABLE_SPRING gate → AC9
**Model:** Cotangent diamond with `disableflags |= DISABLE_SPRING`.
**Expected:** `qfrc_spring` zero for bending DOFs. `qfrc_damper` non-zero
(if velocity perturbation present).
**Source:** Analytically derived (gate logic).

### T10: DISABLE_DAMPER gate → AC10
**Model:** Cotangent diamond with velocity perturbation,
`disableflags |= DISABLE_DAMPER`.
**Expected:** `qfrc_damper` zero. `qfrc_spring` non-zero.
**Source:** Analytically derived (gate logic).

### T11: Cotangent stability — very high stiffness → AC11
**Model:** Same geometry as existing `ac20_bending_stability_clamp` but
with default `bending_model="cotangent"` (no explicit attribute).
`young=1e12`, `thickness=0.005`, `timestep=0.01`.
**Expected:** 500 steps without NaN or blow-up. Cotangent model is inherently
stable (linear force, no nonlinear angle computation).
**Source:** Analytically derived (stability argument: constant B matrix).

### T12: Flap topology — 4x4 grid → AC12
**Model:** 4x4 vertex grid (16 vertices, 18 triangles).
**Expected:** Interior edges have both `flap` entries >= 0. Boundary edges
have `flap[1] == -1`. All diamond stencils valid (4 distinct vertices,
flap vertices not on edge).
**Source:** Analytically derived from grid topology.

### T13: Zero thickness → AC13
**Model:** Equilateral diamond with `thickness=0.0`.
**Expected:** All `flex_bending` entries are zero (`thickness^3 = 0`).
**Source:** Analytically derived.

### T14: Default bending type → AC14
**Model:** Standard dim=2 flex without `bending_model` attribute.
**Expected:** `flex_bending_type[0] == FlexBendingType::Cotangent`.
**Source:** Code review + runtime assertion.

### T15: Mixed bending models → AC15
**Model:** Two flex bodies — one cotangent, one bridson.
**Expected:** Both simulate 100 steps without error. Vertex positions are
finite. No cross-contamination between models.
**Source:** Integration test.

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| Boundary edge (`flap[1] == -1`) | Must produce zero force, not crash | T2 | AC2 |
| Single triangle (no interior edges) | All edges boundary — zero bending | T2 | AC2 |
| Flat rest mesh (`cos_theta=1`, `b[16]=0`) | Common case, verifies formula | T1, T3 | AC1, AC3 |
| Curved rest mesh (`b[16] != 0`) | Non-flat meshes need Garg correction | T4 | AC4 |
| `dim == 1` skip | Cables have no bending | T5 | AC5 |
| `flex_rigid[f]` skip | All-pinned flex skipped | T6 | AC6 |
| `DISABLE_SPRING` / `DISABLE_DAMPER` | Gate flags must be respected | T9, T10 | AC9, AC10 |
| Zero thickness | Produces zero stiffness, zero force | T13 | AC13 |
| Very high stiffness (no clamp) | Cotangent is stable without clamp | T11 | AC11 |
| Pinned vertex in diamond | Force insertion skipped (`dofadr == usize::MAX`) | T7 (strip model has pins at v0, v4) | AC7 |
| Degenerate triangle (zero area) | Guard against division by zero | S2 guard | AC1 (implicit) |

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| T15 (mixed models) | Two bending models in one simulation | Validates dispatch isolation — no AC specifically requires multi-model, but it's the primary risk of the trait architecture |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| Default bending model | Bridson dihedral angle springs | Cotangent Laplacian (MuJoCo-conformant) | **Toward MuJoCo** | Tests asserting specific bending force values | Add `bending_model="bridson"` to preserve old behavior, or update expected values |
| Bending force values | Bridson produces angle-dependent, nonlinear forces | Cotangent produces linear position-dependent forces | **Toward MuJoCo** | `ac6_bending_stiffness` — qualitative test (deflection direction), should still pass | Verify qualitative assertions still hold under cotangent |
| Stability clamp | Per-vertex `fm_max` clamp active for all bending | No clamp for cotangent; clamp preserved for Bridson | **Toward MuJoCo** (MuJoCo has no clamp) | `ac20_bending_stability_clamp` — test becomes Bridson-specific | Gate test on `bending_model="bridson"` |
| New Model fields | No `flex_bending`, `flexedge_flap`, `flex_bending_type` | Three new fields on Model | N/A | Model serialization / `make_data()` | Add field initialization in `make_data()` and `Default` |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `sim/L0/core/src/types/model.rs` | Add `flex_bending: Vec<f64>`, `flexedge_flap: Vec<[i32; 2]>`, `flex_bending_type: Vec<FlexBendingType>`. All derive `Default` (empty vecs). Verify Model `Default` impl or `ModelBuilder::build()` initializes them. No `EXPECTED_SIZE` guard exists for Model (only Data). | +15 |
| `sim/L0/core/src/forward/passive.rs` | Replace Bridson loop (lines 548–670) with per-flex dispatch loop (cotangent + Bridson). Add `apply_bridson_bending()`. | ~+120 / -120 (net ~0, restructured) |
| `sim/L0/mjcf/src/types.rs` | Add `FlexBendingType` enum. Add `bending_model` field to `MjcfFlex`. | +15 |
| `sim/L0/mjcf/src/parser.rs` | Parse `bending_model` attribute from `<elasticity>`. | +10 |
| `sim/L0/mjcf/src/builder/flex.rs` | Add `flexedge_flap` topology computation. Add `compute_bending_coefficients()`, `cot_angle()`, `triangle_area()`. Populate `flex_bending`. Push `flex_bending_type`. | +120 |
| `sim/L0/tests/integration/flex_unified.rs` | New tests T1–T15. Update existing tests as needed. | +400 |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| `ac6_bending_stiffness` | `flex_unified.rs:591` | **Pass (likely unchanged)** | Qualitative test: checks tip_z < root_z and no NaN. Cotangent bending still produces restoring forces. Deflection direction preserved. If specific values change, the test's assertions (directional + no NaN) should still hold. |
| `ac19_bending_damping_only` | `flex_unified.rs:1081` | **Pass (unchanged)** | Tests `young=0` → `k_bend=0`. With cotangent, `young=0` → `mu=0` → `stiffness=0` → B matrix all zeros. Same early-exit behavior. |
| `ac20_bending_stability_clamp` | `flex_unified.rs:1130` | **Pass (value change)** | Under cotangent default, forces are linear (no clamp needed). The test asserts no NaN + finite positions — should pass more easily. The high-stiffness assertion `k_raw > 10000` checks `flex_bend_stiffness` which is still computed for Bridson compatibility. |
| `ac21_single_triangle` | (if exists) | **Pass** | Single triangle = all boundary edges = zero cotangent bending. Same as Bridson behavior. |

### Non-Modification Sites

| File:line | What it does | Why NOT modified |
|-----------|-------------|-----------------|
| `model.rs:81` — `nflexhinge` | Hinge count | Bridson still uses hinges; count unchanged |
| `model.rs:488–492` — `flexhinge_vert/angle0/flexid` | Hinge topology | Bridson still uses these; unchanged |
| `builder/flex.rs:271–315` — hinge extraction | Builds hinge arrays | Still needed for Bridson path |
| `builder/flex.rs:341–365` — `compute_bend_stiffness_from_material` | Computes `flex_bend_stiffness` scalar | Still needed for Bridson path |
| `builder/flex.rs:646–668` — `compute_bend_damping_from_material` | Computes `flex_bend_damping` scalar | Still needed for Bridson path |

---

## Execution Order

**Implementation split:** S1–S4 are build-time (Session 10). S5 is reserved
(absorbed into S2/S3 — see section numbering note). S6–S8 are runtime
(Session 11). Session 11 has a hard dependency on Session 10: the runtime
force loop reads `flex_bending` coefficients that Session 10 populates.

1. **S1: `flexedge_flap` topology** → build-time, no runtime dependency.
   Verify: every interior edge has valid flap vertices. Every boundary edge
   has `flap[1] == -1`. Run T2, T12.

2. **S2+S3: Cotangent precomputation + storage** → depends on S1 (needs
   `flexedge_flap` to identify diamond stencil). Verify: `flex_bending`
   values match EGT-2 for equilateral diamond. Run T1, T4, T13.

3. **S4: `FlexBendingType` enum + MJCF parsing** → independent of S1–S3.
   Can parallelize. Verify: default is `Cotangent`. `bending_model="bridson"`
   parses correctly. Run T14.

4. **--- Session 10 boundary ---** After S1–S4, `flexedge_flap` and
   `flex_bending` are fully populated on Model. This is independently
   testable: verify coefficients without running simulation.

5. **S6: Cotangent runtime force application** → depends on S1–S3 (reads
   `flex_bending` and `flexedge_flap`). Verify: forces match expected
   direction and magnitude. Run T3, T5, T6, T8, T9, T10, T11.

6. **S7: Dispatch architecture (enum match)** → part of S6 (the `match`
   IS the dispatch). No separate implementation step.

7. **S8: `BridsonBending` refactor** → depends on S6 (Bridson is the
   second `match` arm). Verify: Bridson path produces identical results.
   Run T7, T15.

---

## Out of Scope

- **Body-attached flex vertices** (§27D) — Cotangent bending for body-attached
  vertices would need the edge Jacobian (Spec A) for force projection through
  body DOFs. Current scope: free vertices only (3 translational DOFs per
  vertex). Tracked as DT-87 / §27D. *Conformance impact: affects
  body-attached flex models only — no current models use this.*

- **FEM bending for `dim=3`** — MuJoCo uses FEM stiffness (not cotangent
  Laplacian) for volumetric elements. Gated by `dim == 2` check.
  *Conformance impact: none for `dim=2` models.*

- **`elastic2d` keyword** (DT-86) — Fine-grained control over membrane vs
  bending vs nonlinear FEM modes. CortenForge computes bending whenever
  `dim=2 && young>0 && thickness>0`. *Conformance impact: minimal — covers
  default MuJoCo behavior (elastic2d=1 or 3).*

- **Per-edge material variation** — MuJoCo uses per-flex material (not
  per-edge). Same in CortenForge. *Conformance impact: none.*

- **Hinge topology optimization** — Adding `flex_hingeadr`/`flex_hingenum`
  for O(1) hinge iteration in Bridson path. Currently uses filter loop.
  Low priority. *Conformance impact: none (performance only).*

- **GPU flex bending** (DT-67) — Post-v1.0. *Conformance impact: none.*
