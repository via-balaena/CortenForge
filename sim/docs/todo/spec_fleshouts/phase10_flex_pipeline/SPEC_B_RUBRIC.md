# Spec B — Cotangent Laplacian Bending: Spec Quality Rubric

Grades the Spec B spec on 11 criteria. Target: A+ on every criterion
before implementation begins. A+ means "an implementer could build this
without asking a single clarifying question — and the result would produce
numerically identical output to MuJoCo."

**MuJoCo conformance is the cardinal goal.** Every criterion in this rubric
ultimately serves conformance. P1 (MuJoCo Reference Fidelity) is the most
important criterion — grade it first and hardest. A spec that scores A+ on
all other criteria but has P1 wrong is worse than useless: it would produce
a clean, well-tested implementation of the wrong behavior.

> **Extension spec note:** Spec B has a **split mandate**. The cotangent
> Laplacian bending model (Wardetzky/Garg) is the MuJoCo-conformant part
> and must match MuJoCo exactly. The `FlexBendingModel` trait abstraction
> and the `BridsonBending` implementation preserved behind it are
> **CortenForge extensions** — MuJoCo has no trait boundary and no Bridson
> option. The conformance subset (cotangent precomputation + runtime force
> application) must meet the full A+ bar. The extension subset
> (`FlexBendingModel` trait, `BridsonBending` impl, `bending_model` MJCF
> attribute) must be documented as non-MuJoCo with a clear boundary.

Grade scale: A+ (exemplary) / A (solid) / B (gaps) / C (insufficient).
Anything below B does not ship.

---

## Scope Adjustment

| Umbrella claim | MuJoCo reality (C source) | Action |
|----------------|--------------------------|--------|
| `flex_bending`: 17 f64 per edge, flat layout | MuJoCo 3.5.0: `flex_bending` shaped `(nflexedge, 17)` — 17 f64 per edge. `bending[17*e + 4*i + j]` for 4x4 matrix, `bending[17*e + 16]` for curved ref. | In scope, confirmed |
| `flexedge_flap`: 2 vertices per edge (opposite vertices) | MuJoCo 3.5.0: `flex_edgeflap` shaped `(nflexedge, 2)`. `flap[0]` = opposite vertex in tri 1, `flap[1]` = opposite vertex in tri 2 (`-1` for boundary). | In scope, confirmed |
| `flexedge_vert`: 2 vertices per edge (edge endpoints) | Already exists in CortenForge model (`model.rs:440`). Runtime diamond stencil = `flexedge_vert[e]` (v[0],v[1]) + `flexedge_flap[e]` (v[2],v[3]). | In scope (pre-existing) |
| Cotangent weight computation from rest geometry | `ComputeBending()` in `user_mesh.cc:3666-3712`: uses `cot()` helper for cotangent of angles at edge endpoints in both adjacent triangles. 4-element weight vector `c[]`. | In scope, confirmed |
| Material stiffness: `mu = young / (2*(1+poisson))` | `user_mesh.cc:4379`: `mu` passed as `young / (2 * (1 + poisson))`. Stiffness formula: `3 * mu * thickness^3 / (24 * volume)`. | In scope, confirmed |
| 4x4 stiffness matrix outer product | `bending[4*i+j] += c[i] * c[j] * cos_theta * stiffness` | In scope, confirmed |
| Curved reference (Garg correction, b[16]) | `bending[16] = dot(n, e2) * (a01-a03) * (a04-a02) * stiffness / (sqr * sqrt(sqr))` | In scope, confirmed |
| Runtime: matrix-vector spring + damper | `engine_passive.c:245-253`: `spring[3*i+x] += b[4*i+j] * xpos[v[j]*3+x] + b[16] * frc[i][x]`, damper uses same B matrix × velocities | In scope, confirmed |
| `FlexBendingModel` trait | Not in MuJoCo. CortenForge extension for preserving Bridson. | In scope (extension) |
| `bending_model` MJCF attribute | Not in MuJoCo. CortenForge extension attribute on `<flex>`. | In scope (extension) |
| Bending only for `dim == 2` | `engine_passive.c:205`: `if (dim == 2)` gates bending section. dim=3 uses FEM stiffness, dim=1 has no bending. | In scope, confirmed |
| Bending gated on `flex_rigid[f]` | `engine_passive.c:201`: `if (dim == 1 \|\| m->flex_rigid[f]) continue;` | In scope (T1 provides `flex_rigid`) |
| `elastic2d` gate for precomputation | `user_mesh.cc:4374`: `if (dim == 2 && (elastic2d == 1 \|\| elastic2d == 3))` — controls whether bending coefficients are computed. Default shell model uses `elastic2d = 1` (bending only) or `elastic2d = 3` (membrane + bending). | In scope — spec must document how CortenForge triggers bending precomputation |
| Damper force multiplied by `flex_damping[f]` | `engine_passive.c:264`: `d->qfrc_damper[...] -= damper[...] * m->flex_damping[f]` | In scope, confirmed |
| Force insertion uses `body_dofadr` / `body_dofnum` | `engine_passive.c:258-265`: iterates `body_dofnum` DOFs starting at `body_dofadr`. For free vertices, `body_dofnum == 3`. | In scope — currently free-vertex only |

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

## Empirical Ground Truth

### EGT-1: `ComputeBending()` precomputation algorithm

**Source:** `user_mesh.cc:3666-3712` (MuJoCo 3.5.0)

The `ComputeBending()` function computes 17 coefficients per edge for the
cotangent Laplacian bending model. The diamond stencil has 4 vertices:
`v[0], v[1]` (shared edge), `v[2]` (opposite in tri 1), `v[3]` (opposite
in tri 2).

**Step 1: Cotangent weights** via `cot()` helper (`user_mesh.cc:3645-3654`):
```c
double inline cot(const double* x, int v0, int v1, int v2) {
    double edge1[3] = {x[3*v1]-x[3*v0], ...};
    double edge2[3] = {x[3*v2]-x[3*v0], ...};
    double normal[3];
    mjuu_crossvec(normal, edge1, edge2);
    return mjuu_dot3(edge1, edge2) / sqrt(mjuu_dot3(normal, normal));
}
```
Computes `cot(angle at v0)` in triangle `(v0, v1, v2)` = `dot(e1,e2) / |cross(e1,e2)|`.

Four cotangent values:
- `a01 = cot(v[0], v[1], v[2])` — angle at v[0] in tri 1
- `a02 = cot(v[0], v[3], v[1])` — angle at v[0] in tri 2
- `a03 = cot(v[1], v[2], v[0])` — angle at v[1] in tri 1
- `a04 = cot(v[1], v[0], v[3])` — angle at v[1] in tri 2

Weight vector:
```
c[0] = a03 + a04   (sum of cots at v[1] in both tris)
c[1] = a01 + a02   (sum of cots at v[0] in both tris)
c[2] = -(a01 + a03) (negative sum of cots from tri 1)
c[3] = -(a02 + a04) (negative sum of cots from tri 2)
```

**Step 2: Volume** = area of tri 1 + area of tri 2 (diamond area).
`ComputeVolume()` (`user_mesh.cc:3655-3663`):
```c
double inline ComputeVolume(const double* x, const int v[3]) {
    // area = |cross(edge1, edge2)| / 2
    ...
}
```
`volume = ComputeVolume(pos, v) + ComputeVolume(pos, vadj)`
where `v = [v[0], v[1], v[2]]` and `vadj = [v[1], v[0], v[3]]`.

**Step 3: Material stiffness:**
```
stiffness = 3 * mu * thickness^3 / (24 * volume)
```
where `mu = young / (2 * (1 + poisson))` (shear modulus, passed by caller
at `user_mesh.cc:4379`).

**Step 4: cos_theta** (dihedral angle cosine via transport vectors):
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
For a **flat** rest mesh: `cos_theta = 1.0`.
For a **curved** rest mesh: `cos_theta < 1.0`.

**Step 5: 4x4 stiffness matrix:**
```c
for (int v1 = 0; v1 < 4; v1++)
    for (int v2 = 0; v2 < 4; v2++)
        bending[4*v1 + v2] += c[v1] * c[v2] * cos_theta * stiffness;
```

**Step 6: Curved reference coefficient (Garg correction):**
```c
double n[3];
mjuu_crossvec(n, e0, e1);  // normal to tri 1 (unnormalized)
bending[16] = dot(n, e2) * (a01 - a03) * (a04 - a02) * stiffness / (sqr * sqrt(sqr));
```
For a **flat** mesh: `dot(n, e2) = 0` (e2 lies in the plane), so `bending[16] = 0`.
For a **curved** mesh: `dot(n, e2) != 0`, encoding the out-of-plane offset.

### EGT-2: Manual verification — equilateral diamond (flat)

**Geometry:** v0=(0,0,0), v1=(1,0,0), v2=(0.5, √3/2, 0), v3=(0.5, -√3/2, 0).
Two equilateral triangles sharing edge (v0,v1). All edges have length 1.

**Cotangent weights:**
All angles are 60°, so `cot(60°) = 1/√3 ≈ 0.57735`.
- `a01 = a02 = a03 = a04 = 1/√3`
- `c = [2/√3, 2/√3, -2/√3, -2/√3] ≈ [1.1547, 1.1547, -1.1547, -1.1547]`

**Diamond area:** 2 × (√3/4) = √3/2 ≈ 0.8660

**Material (young=1e4, poisson=0.3, thickness=0.01):**
- `mu = 1e4 / 2.6 = 3846.153846`
- `stiffness = 3 × 3846.153846 × 1e-6 / (24 × 0.8660254038) = 5.551444896054e-4`

**cos_theta:**
- `t0 = -(a03×e1 + a01×e3) = -(1/√3×(0.5,0.866,0) + 1/√3×(-0.5,0.866,0)) = (0, -1, 0)`
- `t1 = -(a04×e2 + a02×e4) = -(1/√3×(0.5,-0.866,0) + 1/√3×(-0.5,-0.866,0)) = (0, 1, 0)`
- `sqr = 1.0`, `cos_theta = -dot(t0,t1)/sqr = -(-1)/1 = 1.0` ✓ (flat)

**4x4 matrix:**
`B[i][j] = c[i] × c[j] × 1.0 × stiffness`
= stiffness × (4/3) × sign matrix
where sign matrix = `[[1,1,-1,-1],[1,1,-1,-1],[-1,-1,1,1],[-1,-1,1,1]]`

Each entry magnitude: `stiffness × 4/3 = 7.401926528072e-4`

**Row sums:** All zero — this is a fundamental property of the cotangent
Laplacian (`c[0]+c[1]+c[2]+c[3] = 0`, so each row of the outer product
sums to zero). Verified numerically.

**Curved reference:** `bending[16] = 0` (flat mesh). ✓

### EGT-2b: Manual verification — asymmetric curved diamond

**Geometry:** v0=(0,0,0), v1=(2,0,0), v2=(0.5,1,0), v3=(1.5,-0.8,0.5).
Asymmetric triangles, v3 out of plane. This verifies that the formulas
work for non-symmetric, non-flat geometries.

**Cotangent weights:**
- `a01=0.500, a02=1.590, a03=1.500, a04=0.530` (all different)
- `(a01-a03) = -1.0`, `(a04-a02) = -1.060` (non-zero → b[16] non-zero)

**Results:**
- `cos_theta = 0.8480` (< 1, curved mesh)
- `b[16] = 3.278e-5` (non-zero, encoding rest curvature)
- Row sums: zero (cotangent Laplacian property preserved)

This confirms the formulas handle asymmetric, curved geometries correctly.
The `b[16]` curved reference term only activates when both:
(1) the mesh is non-flat (`dot(n, e2) != 0`), AND
(2) the triangles have different cotangent distributions at the edge
endpoints (`(a01-a03) != 0` or `(a04-a02) != 0`).

### EGT-3: Runtime bending force application

**Source:** `engine_passive.c:192-268` (MuJoCo 3.5.0)

Gate conditions:
1. `dim == 1 || flex_rigid[f]` → skip (`engine_passive.c:201`)
2. `dim == 2` → bending section (`engine_passive.c:205`)
3. `v[3] == -1` → skip boundary edge (`engine_passive.c:215`)
4. `has_spring` / `has_damping` gates on individual force components

**Edge vectors (runtime, from current positions):**
```c
mji_sub3(ed[0], xpos + 3*v[1], xpos + 3*v[0]);  // v1-v0
mji_sub3(ed[1], xpos + 3*v[2], xpos + 3*v[0]);  // v2-v0
mji_sub3(ed[2], xpos + 3*v[3], xpos + 3*v[0]);  // v3-v0
```

**Curved reference forces (cross products):**
```c
frc[1] = ed[1] × ed[2]  // (v2-v0) × (v3-v0)
frc[2] = ed[2] × ed[0]  // (v3-v0) × (v1-v0)
frc[3] = ed[0] × ed[1]  // (v1-v0) × (v2-v0)
frc[0] = -(frc[1] + frc[2] + frc[3])
```

**Force accumulation (triple-nested loop):**
```c
for i in 0..4:
    for x in 0..3:
        for j in 0..4:
            spring[3*i+x] += b[17*e+4*i+j] * xpos[3*v[j]+x]  // B × pos
            damper[3*i+x] += b[17*e+4*i+j] * vel[j][x]        // B × vel
        spring[3*i+x] += b[17*e+16] * frc[i][x]                // curved ref
```

**Global force insertion:**
```c
for i in 0..4:
    d->qfrc_spring[body_dofadr + x] -= spring[3*i + x];
    d->qfrc_damper[body_dofadr + x] -= damper[3*i + x] * flex_damping[f];
```

Key observations:
- Spring forces are **subtracted** from `qfrc_spring` (negative sign)
- Damper forces are **subtracted** AND multiplied by `flex_damping[f]`
- Velocities read from `d->qvel + body_dofadr[bodyid[v[i]]]`
- Force written to `body_dofadr` with `body_dofnum` DOFs (3 for free vertices)
- The bending stiffness is already baked into `b[]` coefficients — no runtime
  multiplication by `flex_bend_stiffness`
- `flex_damping[f]` is the proportional damping coefficient (separate from
  bending stiffness, which is encoded in the B matrix)

### EGT-4: `CreateFlapStencil()` topology

**Source:** `user_mesh.cc:3593-3639` (MuJoCo 3.5.0)

Builds `flaps` vector from element connectivity:
- For each triangle, extracts 3 edges (local numbering `{{1,2}, {2,0}, {0,1}}`)
- Creates canonical edge key: `(min(v_a, v_b), max(v_a, v_b))`
- First encounter: creates `StencilFlap` with `vertices = [edge[0], edge[1], opposite, -1]`
- Second encounter: fills `vertices[3]` with the opposite vertex from the second triangle
- Boundary edges retain `vertices[3] = -1`

**Vertex ordering in flap stencil:**
- `flap.vertices[0]` = first edge vertex (from the local edge definition)
- `flap.vertices[1]` = second edge vertex
- `flap.vertices[2]` = opposite vertex in the first triangle encountered
- `flap.vertices[3]` = opposite vertex in the second triangle (`-1` if boundary)

**Important:** The edge vertex ordering depends on which triangle is encountered
first. The `cot()` computation in `ComputeBending()` is symmetric with respect
to which triangle is "tri 1" vs "tri 2" — swapping them produces identical
weight vectors and stiffness matrix.

### EGT-5: Gate conditions and elastic2d

**Precomputation gate** (`user_mesh.cc:4374`):
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

CortenForge always computes bending for `dim == 2` flex with `thickness > 0`
(matching the default MuJoCo behavior with the shell plugin). The `elastic2d`
concept maps to whether bending coefficients are populated at build time.

**Runtime gate** (`engine_passive.c:192-205`):
```c
for (int f = 0; f < m->nflex; f++) {
    mjtNum* b = m->flex_bending + 17*m->flex_edgeadr[f];
    int dim = m->flex_dim[f];
    if (dim == 1 || m->flex_rigid[f]) continue;  // skip cables and rigid
    if (dim == 2) {
        // bending section
    }
}
```
No separate `has_bending` flag — if `flex_bending` coefficients are all zero
(boundary edges or no bending computed), the matrix-vector multiply produces
zero forces naturally.

### EGT-6: Bridson bending (current implementation)

**Source:** `sim/L0/core/src/forward/passive.rs:548-670`

Current Bridson implementation:
- Iterates `nflexhinge` hinges (not edges)
- Uses `flexhinge_vert: Vec<[usize; 4]>` for topology (`[ve0, ve1, va, vb]`)
- Uses `flexhinge_angle0: Vec<f64>` for rest dihedral angles
- Computes dihedral angle via `atan2(sin_theta, cos_theta)` each step
- Bridson gradient computation for all 4 vertices
- Per-vertex stability clamp: `fm_max = 1/(dt² × |grad| × invmass)`
- Spring forces → `qfrc_spring`, damper forces → `qfrc_damper`
- Uses `flex_bend_stiffness[flex_id]` and `flex_bend_damping[flex_id]`

The Bridson model will be preserved in `BridsonBending` trait impl with
no algorithm changes. The current hinge topology (`flexhinge_*`) remains
for Bridson; the new edge topology (`flexedge_flap`, `flex_bending`) is
for cotangent only.

### EGT-7: Codebase context — files and match sites

| File | Lines | What changes | Spec section |
|------|-------|-------------|-------------|
| `sim/L0/core/src/forward/passive.rs` | 548-670 | Replace Bridson loop with trait-dispatched bending. Add `FlexBendingModel` trait, `CotangentBending`, `BridsonBending` impls. | S6, S7, S8 |
| `sim/L0/core/src/types/model.rs` | ~330, ~488-492 | Add `flex_bending: Vec<f64>`, `flexedge_flap: Vec<[i32; 2]>`, `flex_bending_type: Vec<FlexBendingType>`. Existing: `flex_bend_stiffness`, `flex_bend_damping`, `flexhinge_*` (keep for Bridson). | S1-S5 (storage) |
| `sim/L0/core/src/types/data.rs` | ~127-136 | No new Data fields for bending (precomputed in Model). | — |
| `sim/L0/mjcf/src/builder/flex.rs` | 271-365 | Add `flexedge_flap` topology computation from element connectivity. Add `flex_bending` precomputation (cotangent weights, stiffness matrix, Garg correction). Keep existing hinge extraction for Bridson. | S1-S5 (computation) |
| `sim/L0/mjcf/src/types.rs` | ~3760-3858 | Add `FlexBendingType` enum. Add `bending_model` field to `MjcfFlex`. | S7 (type definition) |
| `sim/L0/mjcf/src/parser.rs` | ~2801-2805 | Parse `bending_model` attribute from `<elasticity>` or `<flex>` element. | S7 (parsing) |
| `sim/L0/tests/integration/flex_unified.rs` | 591-630, 1081-1175 | Existing bending tests: `ac6_bending_stiffness`, `ac19_bending_damping_only`, `ac20_bending_stability_clamp`, `ac21_single_triangle`. Must pass with Bridson; new cotangent tests added. | All |

**Match sites for `flexhinge_*`** (must NOT be removed — Bridson uses them):
- `model.rs:81` — `nflexhinge: usize`
- `model.rs:488-492` — `flexhinge_vert`, `flexhinge_angle0`, `flexhinge_flexid`
- `passive.rs:552` — `for h in 0..model.nflexhinge`
- `builder/flex.rs:271-315` — hinge extraction from element adjacency

**Match sites for existing bending stiffness computation:**
- `builder/flex.rs:341-365` — `compute_bend_stiffness_from_material()`,
  `compute_bend_damping_from_material()` — these compute the scalar
  `flex_bend_stiffness` and `flex_bend_damping` for the Bridson model.
  The cotangent model bakes stiffness into the B matrix, but still
  needs `flex_damping[f]` for the damper multiplier.

---

## Criteria

> **Criterion priority:** P1 (MuJoCo Reference Fidelity) is the cardinal
> criterion. Grade P1 first and grade it hardest.

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does — exact function names, field
> names, calling conventions, and edge cases. No hand-waving.

| Grade | Bar |
|-------|-----|
| **A+** | Every MuJoCo function/field/flag cited with source file, line range, and exact behavior. Specifically: `ComputeBending()` (`user_mesh.cc:3666-3712`), `cot()` (`user_mesh.cc:3645-3654`), `ComputeVolume()` (`user_mesh.cc:3655-3663`), `CreateFlapStencil()` (`user_mesh.cc:3593-3639`), and runtime bending loop (`engine_passive.c:192-268`). Cotangent weight formula (`c[0..4]`) stated with exact index mapping. cos_theta transport vector formula stated. Garg curved reference formula (`bending[16]`) stated with derivation. Edge cases addressed: boundary edges (`flap[1]==-1`), flat rest mesh (`cos_theta=1`, `b[16]=0`), `dim==1` skip, `dim==3` skip, `flex_rigid[f]` skip, `has_spring`/`has_damping` gates, degenerate triangles (zero area). Force sign convention: spring **subtracted** from `qfrc_spring`, damper **subtracted** and **multiplied by `flex_damping[f]`**. Velocity source: `d->qvel + body_dofadr[bodyid[v[i]]]`. C code snippets included for all non-obvious formulas. |
| **A** | MuJoCo behavior described correctly from C source. Minor gaps in edge-case coverage (e.g., degenerate triangle handling). |
| **B** | Correct at high level, but missing specifics — e.g., force sign convention wrong, or damper missing `flex_damping[f]` multiplier, or cos_theta formula omitted. |
| **C** | Partially correct. Algorithm described from docs/papers rather than C source. |

> **P1 / P9 boundary:** P1 grades whether the spec correctly describes
> what MuJoCo computes (the "what"). P9 (Geometric Correctness) grades
> whether the mathematical derivation of cotangent weights and transport
> vectors is rigorous (the "why").

### P2. Algorithm Completeness

> Every algorithmic step is specified unambiguously. No "see MuJoCo source"
> gaps.

| Grade | Bar |
|-------|-----|
| **A+** | Every loop, every formula, every guard is written out in Rust. An implementer can type it in without reading MuJoCo source. Specifically: (1) `cot()` function in Rust, (2) `c[0..4]` weight computation, (3) `ComputeVolume` (triangle area), (4) material stiffness formula, (5) cos_theta via transport vectors, (6) 4x4 outer product loop, (7) Garg b[16] formula, (8) boundary edge guard, (9) runtime triple-nested force loop, (10) curved reference cross-product forces, (11) force insertion with sign convention and damping multiplier, (12) `CreateFlapStencil` edge-to-triangle adjacency algorithm. No "verify against source" notes. |
| **A** | Algorithm is complete. One or two minor details left implicit (e.g., edge vector variable names). |
| **B** | Algorithm structure clear but some steps hand-waved (e.g., "compute cotangent weights per Wardetzky"). |
| **C** | Skeleton only. |

### P3. Convention Awareness

> Spec explicitly addresses our codebase's conventions where they differ
> from MuJoCo.

| Grade | Bar |
|-------|-----|
| **A+** | Every MuJoCo → CortenForge translation documented: (1) `int -1` sentinel → `usize::MAX` or `i32 -1` for boundary flap vertices, (2) `body_dofadr[bodyid[v[i]]]` → `flexvert_dofadr[v[i]]` for free vertices (equivalent for free-vertex case), (3) `flex_bending` flat indexing `17*e + 4*i + j` → Rust `Vec<f64>` with same layout, (4) MuJoCo `flex_damping[f]` → CortenForge field naming, (5) `qfrc_spring` / `qfrc_damper` accumulation convention (S4.7-prereq from Phase 5), (6) `has_spring` / `has_damping` → `DISABLE_SPRING` / `DISABLE_DAMPER` flags, (7) MuJoCo `flex_edgeflap` per-edge 2-element → CortenForge `flexedge_flap: Vec<[i32; 2]>`. Convention difference table present with verified porting rules. |
| **A** | Major conventions documented. Minor field-name mappings left to implementer. |
| **B** | Some conventions noted, others not. |
| **C** | MuJoCo code pasted without adaptation. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable. Contains concrete values
> and tolerances.

| Grade | Bar |
|-------|-----|
| **A+** | Every runtime AC has: (1) concrete input model/state, (2) exact expected value or tolerance, (3) what field to check. At least one AC per major feature uses values from the EGT-2 equilateral diamond manual verification (or another hand-computable geometry). Cotangent precomputation AC specifies expected `flex_bending` values for the equilateral diamond (EGT-2: `c = [2/√3, 2/√3, -2/√3, -2/√3]`, stiffness ≈ 5.551e-4, `b[16] = 0`). Runtime force AC specifies expected `qfrc_spring` direction and magnitude for a deflected diamond. Bridson regression AC specifies exact match with pre-Spec-B values. Stability AC specifies 500 steps without force clamp for `young=1e12`. Code-review ACs explicitly labeled. Tolerance: `1e-10` for precomputation (exact arithmetic), `1e-8` for runtime (floating-point accumulation). |
| **A** | ACs are testable. Some lack exact numerical expectations. |
| **B** | ACs are directionally correct but vague ("bending forces should be correct"). |
| **C** | ACs are aspirational. |

### P5. Test Plan Coverage

> Tests cover happy path, edge cases, regressions, and interactions.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present. Edge case inventory: boundary edges (flap[1]==-1), single triangle (no bending), flat rest mesh (b[16]=0), curved rest mesh (b[16]≠0), degenerate triangle (zero area → zero/inf stiffness guard), `dim==1` skip, `flex_rigid` skip, `DISABLE_SPRING` / `DISABLE_DAMPER` flags, zero thickness (error or skip), very high stiffness (stability without clamp). Negative cases: boundary edge produces zero force, dim=1 flex produces zero bending, rigid flex skipped. Conformance test: equilateral diamond with known cotangent weights and B matrix values. Regression test: Bridson path produces identical results to pre-Spec-B (`bending_model="bridson"`). Mixed-model test: two flex bodies with different bending models. At least one non-trivial model (4x4 grid shell). Supplementary tests justified. |
| **A** | Good coverage. Minor edge-case gaps. |
| **B** | Happy path covered. Edge cases sparse. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering constraints, and interactions stated.

| Grade | Bar |
|-------|-----|
| **A+** | Execution order unambiguous. Implementation split clearly documented: S1-S5 (build-time, Session 10) vs S6-S8 (runtime, Session 11). Each section states what it requires from prior sections. Cross-spec interactions with T1 (`flex_rigid` for gate), Spec A (soft dep: edge Jacobian for body-attached vertices), and umbrella conventions (force application pattern, field naming) called out. Prerequisites include T1 commit hash for `flex_rigid`. S6 requires S1-S5 complete (hard internal dependency). |
| **A** | Order is clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Spec identifies every file touched, every behavior that changes.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list with per-file change description (see EGT-7). Behavioral changes documented: (1) default bending model changes from Bridson to Cotangent — this is a **conformance-improving** change. (2) Cotangent bending produces different force values than Bridson for the same material properties — any test asserting specific bending force values will see different numbers. (3) Bridson regression path available via `bending_model="bridson"`. Existing test impact: `ac6_bending_stiffness` (line 591) — values change under cotangent default, needs update or `bending_model` gate. `ac19_bending_damping_only`, `ac20_bending_stability_clamp` — Bridson-specific, should pass with `bending_model="bridson"`. New `flex_bending` field on Model triggers `EXPECTED_SIZE` guard update (if present). Backward-compat: models without `bending_model` attribute get `Cotangent` (conformant default). |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions within the spec.

| Grade | Bar |
|-------|-----|
| **A+** | Terminology is uniform: "diamond stencil" used consistently for the 4-vertex configuration, "cotangent" vs "Bridson" never confused, `flex_bending` vs `flex_bend_stiffness` distinguished (B matrix vs scalar). Cross-references accurate: S1-S5 section numbers match between Specification and Test Plan. File paths match between Specification and Files Affected. AC numbers match between AC section and Traceability Matrix. Edge case list consistent between MuJoCo Reference and Test Plan. Consumer count (bending section in `passive.rs`) consistent. |
| **A** | Consistent. One or two minor terminology issues. |
| **B** | Some sections use different names for same concept. |
| **C** | Contradictions between sections. |

### P9. Geometric Correctness

> Mathematical derivation of cotangent weights, transport vectors, and Garg
> correction is rigorous and verifiable.

| Grade | Bar |
|-------|-----|
| **A+** | Cotangent weight formula derived from Wardetzky operator with geometric meaning explained (cotangent of angles at edge endpoints). The `cot()` function's geometric interpretation stated: `dot(e1,e2)/|cross(e1,e2)| = cos(α)/sin(α) = cot(α)`. Transport vector formula (`t0`, `t1`) derived from the parallel transport of edge normals within each triangle. cos_theta geometric meaning: cosine of the angle between transported normals (= 1 for flat, < 1 for curved). Garg correction `b[16]` geometric meaning: encodes rest curvature via the signed volume `dot(n, e2)` where `n = cross(e0, e1)`. Manual verification (EGT-2) confirms all formulas produce expected values for equilateral diamond. At least one non-equilateral example verifies the formulas are not accidentally symmetric-geometry-only correct. |
| **A** | Geometric derivation correct. Minor gaps in explanation. |
| **B** | Formulas stated but not derived. Reader must trust they're correct. |
| **C** | Formulas copied from paper without verification. |

### P10. Trait Architecture

> `FlexBendingModel` trait design is clean, zero-overhead, and establishes
> the pattern for post-v1.0 trait architecture.

| Grade | Bar |
|-------|-----|
| **A+** | Trait signature specified with exact Rust types. Two implementations (`CotangentBending`, `BridsonBending`) described with their precomputation requirements and runtime contracts. Dispatch mechanism from `mj_fwd_passive()` specified (match on `FlexBendingType` enum, NOT dynamic dispatch — zero-overhead). `FlexBendingType` enum with `Cotangent` (default, MuJoCo-conformant) and `Bridson` variants. Extension boundary clearly marked: MuJoCo has no trait, no Bridson option, no `bending_model` attribute. The trait does not leak into any MuJoCo-conformant code path — `CotangentBending` must produce identical results whether called through the trait or inlined. MJCF `bending_model` attribute parsing specified (extension attribute, not in MuJoCo schema). |
| **A** | Trait design is clean. Minor details left to implementer. |
| **B** | Trait exists but dispatch mechanism unclear. |
| **C** | Trait is afterthought or over-engineered. |

### P11. Precomputation-Runtime Seam

> The build-time / runtime boundary is clearly defined and the split
> implementation (Session 10 vs Session 11) produces a testable
> intermediate state.

| Grade | Bar |
|-------|-----|
| **A+** | Seam explicitly documented: S1-S5 deliver `flexedge_flap` and `flex_bending` fully populated on Model at build time. S6-S8 deliver runtime force application that reads those fields. After Session 10 (S1-S5), the build-time state is independently testable: `flex_bending` values can be verified against EGT-2 manual computation without running any simulation. Session 11 (S6-S8) has a hard dependency on Session 10 output. The seam matches MuJoCo's own split: `user_mesh.cc` (compile time) vs `engine_passive.c` (runtime). Test plan includes at least one build-time-only test (verify `flex_bending` coefficients) and at least one runtime test (verify `qfrc_spring` after forward). |
| **A** | Seam is clear. Testability of intermediate state addressed. |
| **B** | Split mentioned but intermediate testability not addressed. |
| **C** | No discussion of implementation split. |

---

## Rubric Self-Audit

### Self-audit checklist

- [x] **Specificity:** Every A+ bar names specific MuJoCo functions
      (`ComputeBending`, `cot`, `ComputeVolume`, `CreateFlapStencil`,
      `engine_passive.c` bending loop), specific formulas (`c[0..4]`,
      cos_theta, `b[16]`), specific edge cases (boundary, flat, curved,
      degenerate, dim=1, rigid), and specific CortenForge files/lines
      (`passive.rs:548-670`, `builder/flex.rs:271-365`). Two independent
      reviewers would agree on the grade by pointing to spec content.

- [x] **Non-overlap:** P1 vs P9 boundary: P1 grades whether the spec
      correctly describes what MuJoCo computes (the reference); P9 grades
      whether the mathematical derivation is rigorous (the geometry).
      P1 vs P10: P1 covers MuJoCo-conformant behavior; P10 covers the
      extension trait architecture. P7 vs P11: P7 covers file-level blast
      radius; P11 covers the implementation split boundary.

- [x] **Completeness:** 11 criteria cover: MuJoCo reference (P1),
      algorithm (P2), conventions (P3), ACs (P4), tests (P5),
      dependencies (P6), blast radius (P7), consistency (P8),
      geometry (P9), trait design (P10), impl split (P11). No meaningful
      gap remains — a spec A+ on all 11 covers every dimension.

- [x] **Gradeability:** P1 → MuJoCo Reference + Key Behaviors;
      P2 → Specification (S1-S8); P3 → Convention Notes; P4 → ACs;
      P5 → Test Plan + Traceability; P6 → Prerequisites + Execution Order;
      P7 → Files Affected + Behavioral Changes; P8 → cross-cutting;
      P9 → MuJoCo Reference geometric derivations; P10 → trait definition
      sections; P11 → Execution Order split marking.

- [x] **Conformance primacy:** P1 is tailored with 5 specific MuJoCo C
      functions, exact line ranges, and 7 edge cases. P4 requires
      MuJoCo-derived expected values (EGT-2 manual verification). P5
      requires conformance tests with hand-verified values. P9 requires
      geometric derivation verified against manual computation.

- [x] **Empirical grounding:** 8 EGT sections (EGT-1 through EGT-7 plus
      EGT-2b) filled in from C source analysis. EGT-2 provides manual
      verification for equilateral diamond; EGT-2b provides asymmetric
      curved verification. Every A+ bar referencing MuJoCo behavior has
      a corresponding EGT entry.

### Criterion → Spec section mapping

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P1 | MuJoCo Reference, Key Behaviors table, Convention Notes |
| P2 | Specification (S1-S8) |
| P3 | Convention Notes, Specification code |
| P4 | Acceptance Criteria |
| P5 | Test Plan, AC→Test Traceability Matrix, Edge Case Inventory |
| P6 | Prerequisites, Execution Order |
| P7 | Risk & Blast Radius (Behavioral Changes, Files Affected, Existing Test Impact) |
| P8 | *Cross-cutting — all sections checked for mutual consistency* |
| P9 | MuJoCo Reference geometric derivations, Specification S2-S5 formulas |
| P10 | Specification S7-S8, trait definition, dispatch mechanism |
| P11 | Execution Order split marking, S1-S5 vs S6-S8 boundary |

---

## Scorecard

| Criterion | Grade | Evidence |
|-----------|-------|----------|
| P1. MuJoCo Reference Fidelity | — | *(grade after spec is written)* |
| P2. Algorithm Completeness | — | |
| P3. Convention Awareness | — | |
| P4. Acceptance Criteria Rigor | — | |
| P5. Test Plan Coverage | — | |
| P6. Dependency Clarity | — | |
| P7. Blast Radius & Risk | — | |
| P8. Internal Consistency | — | |
| P9. Geometric Correctness | — | |
| P10. Trait Architecture | — | |
| P11. Precomputation-Runtime Seam | — | |

**Overall: Rubric Rev 2 (stress-tested, ready for spec grading)**

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| R1 | P1 | Initial draft lacked `elastic2d` gate condition for precomputation | C source review (`user_mesh.cc:4374`) | Added EGT-5 documenting `elastic2d` modes and mapping to CortenForge | Rubric Rev 1 |
| R2 | P1 | Damper force sign and `flex_damping[f]` multiplier not in initial A+ bar | C source review (`engine_passive.c:264`) | Added to P1 A+ bar: "subtracted AND multiplied by `flex_damping[f]`" | Rubric Rev 1 |
| R3 | P3 | Missing `body_dofadr` vs `flexvert_dofadr` convention difference | C source review (`engine_passive.c:258`) | Added to P3 A+ bar: MuJoCo uses `body_dofadr[bodyid[v[i]]]`, CortenForge uses `flexvert_dofadr[v[i]]` for free vertices | Rubric Rev 1 |
| R4 | P9 | No domain-specific criterion for geometric correctness of cotangent derivation | Self-audit: P1 covers "what MuJoCo does" but not "why the math is correct" | Added P9 (Geometric Correctness) | Rubric Rev 1 |
| R5 | P10 | No domain-specific criterion for trait architecture (first trait in pipeline) | Self-audit: this is a key Phase 10 structural deliverable | Added P10 (Trait Architecture) | Rubric Rev 1 |
| R6 | P11 | No criterion for implementation split boundary (Session 10/11 seam) | Self-audit: split impl is a key Phase 10 structural feature | Added P11 (Precomputation-Runtime Seam) | Rubric Rev 1 |
| R7 | EGT | Missing manual verification values for non-equilateral geometry | Self-audit: equilateral-only verification could miss asymmetric bugs | Noted in P9 A+ bar: "at least one non-equilateral example" required in spec | Rubric Rev 1 |
| R8 | P1 | Initial draft didn't distinguish `flex_bend_stiffness` (Bridson scalar) from `flex_bending` (cotangent matrix) | Codebase context review | Added to EGT-7 and P8: these are distinct fields for different models | Rubric Rev 1 |
| R9 | P7 | Missing EXPECTED_SIZE guard impact | Codebase context review | Added to P7 A+ bar: new Model fields may trigger staleness guards | Rubric Rev 1 |
| R10 | P1 | P1/P9 boundary note incorrectly referenced "P10 (Geometric Correctness)" — P10 is Trait Architecture, P9 is Geometric Correctness | Stress test | Fixed reference from P10 to P9 | Rubric Rev 2 |
| R11 | P4 | Equilateral diamond stiffness value rounded to 5.553e-4 instead of correct 5.551e-4 | Stress test arithmetic check | Fixed to 5.551e-4 (matches EGT-2: 5.551444896054e-4) | Rubric Rev 2 |
| R12 | P1 | Scope table mentioned `flexedge_flap` but not `flexedge_vert` — runtime needs both to reconstruct the 4-vertex diamond stencil | Stress test: traced runtime v[0..3] source | Added `flexedge_vert` to scope table as pre-existing field | Rubric Rev 2 |
