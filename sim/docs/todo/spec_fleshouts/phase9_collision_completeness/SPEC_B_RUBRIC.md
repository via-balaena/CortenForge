# Spec B — Mesh Inertia Modes: Spec Quality Rubric

Grades the Spec B spec on 10 criteria. Target: A+ on every criterion
before implementation begins. A+ means "an implementer could build this
without asking a single clarifying question — and the result would produce
numerically identical output to MuJoCo."

**MuJoCo conformance is the cardinal goal.** Every criterion in this rubric
ultimately serves conformance. P1 (MuJoCo Reference Fidelity) is the most
important criterion — grade it first and hardest. A spec that scores A+ on
all other criteria but has P1 wrong is worse than useless: it would produce
a clean, well-tested implementation of the wrong behavior.

Grade scale: A+ (exemplary) / A (solid) / B (gaps) / C (insufficient).
Anything below B does not ship.

---

## Scope Adjustment

Phase 9 Umbrella and `future_work_11.md` (§43) defined the original scope.
Empirical verification against MuJoCo 3.5.0 discovered the following
corrections:

| Umbrella claim | MuJoCo 3.5.0 reality | Action |
|----------------|----------------------|--------|
| Default mesh inertia is `exact` | Default is `convex` (enum value 0 in `mjtMeshInertia`). For convex meshes like cubes, `convex` == `exact`, masking the difference. | **Correct:** default = `Convex` |
| `<compiler exactmeshinertia>` exists (deprecated) | **Removed entirely** in MuJoCo 3.5.0 — XML parser rejects it as "unrecognized attribute" | **Drop:** remove `exactmeshinertia` from `MjcfCompiler` (or keep as ignored legacy parse) |
| `shellinertia` on `<geom>` is a `bool` attribute | MJCF XML attribute is `shellinertia="true/false"` (boolean). C struct is `mjtGeomInertia typeinertia` (enum: `VOLUME`/`SHELL`). For mesh geoms, MuJoCo 3.5.0 **rejects** `shellinertia` with error: "for mesh geoms, inertia should be specified in the mesh asset" | **In scope:** parse as `Option<bool>`, reject on mesh geoms |
| `shellinertia` is backward compat for `<mesh inertia="shell">` | `shellinertia` on `<geom>` ONLY applies to primitives. It does NOT set mesh inertia mode. `<mesh inertia="shell">` is the only way to get shell mode for meshes. | **Correct** |
| `<mesh inertia="...">` not inheritable via default class | `<default><mesh inertia="shell"/>` IS supported — mesh defaults inherit `inertia` attribute | **In scope:** support via `MjcfMeshDefaults` |
| Primitive shell formulas: sphere, box, capsule, cylinder, ellipsoid | Empirically verified: all 5 produce shell inertia via `shellinertia="true"` | **In scope:** all 5 primitives |
| Legacy mode: backward compat algorithm | `inertia="legacy"` accepted; produces different values from exact on non-convex meshes but same on convex. Uses absolute tetrahedron volumes (overcounting). | **In scope** |

**Final scope:**

1. `MeshInertia` enum (`Convex`, `Exact`, `Legacy`, `Shell`) on `MjcfMesh`
2. `inertia` attribute parsing on `<mesh>` element (default: `Convex`)
3. `inertia` inheritance via `<default><mesh/>`
4. Shell mesh inertia algorithm (area-weighted surface distribution, divisor 12)
5. Convex mesh inertia (exact algorithm on Spec A's convex hull)
6. Legacy mesh inertia (absolute tetrahedron volumes — overcounting)
7. `shellinertia` attribute on `<geom>` for primitive shell inertia
8. Primitive shell inertia formulas (sphere, box, capsule, cylinder, ellipsoid)
9. `shellinertia` rejected on mesh geoms (error: use `<mesh inertia="...">`)
10. `exactmeshinertia` deprecation handling (already parsed; decide: keep or remove)

---

## Empirical Ground Truth

### EGT-1: Default mesh inertia mode is `convex`

**MuJoCo 3.5.0, Python.** Unit cube (`-0.5..0.5`), density 1000, no
`inertia` attribute specified:

```
body_mass[1] = 1000.0
body_inertia[1] = [166.667, 166.667, 166.667]
```

Same result as `inertia="convex"` and `inertia="exact"` (cube is its own
convex hull). Default == convex confirmed.

**C source:** `mjspec.h` line 67: `mjMESH_INERTIA_CONVEX = 0` (first enum
value). Zero-initialized struct fields default to this.

### EGT-2: Shell mesh inertia — unit cube

**MuJoCo 3.5.0.** Unit cube with `inertia="shell"`, density 1000:

```
body_mass[1] = 6000.0       // density * SA = 1000 * 6.0
body_inertia[1] = [1666.667, 1666.667, 1666.667]
```

Surface area of unit cube = 6 × 1² = 6.0 (12 triangles × 0.5 area each).
Shell inertia ratio: I/m = 1666.67/6000 = 5/18 ≈ 0.2778 (for unit cube
with edge length 1). Note: this ratio uses full edge length, not MuJoCo
half-extents.

### EGT-3: Shell mesh inertia — explicit mass override

Unit cube with `inertia="shell"` on mesh, `mass="5.0"` on geom:

```
body_mass[1] = 5.0
body_inertia[1] = [1.389, 1.389, 1.389]
```

Scale factor = 5.0/6000 = 1/1200. Inertia scales proportionally: 1666.67/1200 = 1.389. ✓

### EGT-4: Primitive shell inertia — sphere

Sphere, r=1, density=1000, `shellinertia="true"`:

```
body_mass[1] = 12566.37     // density * 4πr² = 1000 * 12.566
body_inertia[1] = [8377.58, 8377.58, 8377.58]
```

Analytical: I = 2/3 · m · r² = 2/3 × 12566.37 × 1 = 8377.58 ✓

Solid sphere (default): mass = 4188.79 (density × 4/3πr³), I = 1675.52 (2/5·m·r²).

### EGT-5: Primitive shell inertia — box

Box, half-extents (1,2,3), density=1000, `shellinertia="true"`:

```
body_mass[1] = 88000.0      // density * SA = 1000 * 88
body_inertia[1] = [541333.33, 421333.33, 242666.67]
```

**WARNING:** The simple formula `m/6*(b²+c²)` from `future_work_11.md` (§43)
produces Ix=762666.67 — **WRONG**. The correct algorithm is face-by-face
decomposition: 6 thin rectangular plates (one per face), each with its own
moment of inertia computed analytically, then shifted to box center via
parallel axis theorem. This face decomposition matches MuJoCo exactly.

Derivation: For a box with half-extents (a,b,c), the ±x faces have area
2b×2c, mass_face = density×4bc, I_face_yy = m_face/12×(2c)² + m_face×a²
(PAT by distance a). Sum all 6 faces. Verified: Ix=541333.33 ✓.

### EGT-6: Primitive shell inertia — cylinder

Cylinder, r=1, half-h=2, density=1000, `shellinertia="true"`:

```
body_mass[1] = 31415.93     // density * SA (lateral + 2 caps)
body_inertia[1] = [72780.23, 72780.23, 28274.33]
```

SA = 2πrh + 2πr² = 2π(4) + 2π = 10π ≈ 31.416. Mass = 31415.93 ✓.

### EGT-7: Primitive shell inertia — capsule

Capsule, r=1, half-h=2, density=1000, `shellinertia="true"`:

```
body_mass[1] = 37699.11     // density * SA (cylinder + sphere)
body_inertia[1] = [129852.50, 129852.50, 33510.32]
```

SA = 2πrh + 4πr² = 8π + 4π = 12π ≈ 37.699. Mass = 37699.11 ✓.

**Derivation:** Capsule shell = cylindrical lateral shell + two hemisphere
shells. Cylinder lateral: m_cyl = density×2πrh, Ix_cyl = m_cyl×(r²/2 + h²/12).
Hemisphere shell: m_hemi = density×2πr², Ix_hemi_about_own_com = m_hemi×r²×5/12.
COM of hemisphere is at distance r/2 from equator, so PAT distance to capsule
center is d = h/2 + r/2. Ix_hemi_shifted = Ix_hemi_about_own_com + m_hemi×d².
Total Ix = Ix_cyl + 2×Ix_hemi_shifted = 129852.50 ✓.

**WARNING:** Naive PAT from sphere center to capsule center (d = h/2 + 0)
gives Ix=104719.76 — **WRONG**. Must use hemisphere COM offset (r/2).

### EGT-8: Primitive shell inertia — ellipsoid

Ellipsoid, semi-axes (1,2,3), density=1000, `shellinertia="true"`:

```
body_mass[1] = 48971.93
body_inertia[1] = [180751.03, 140683.07, 81026.34]
```

Ellipsoid surface area uses Knud Thomsen's approximation: SA ≈
4π((a^p·b^p + a^p·c^p + b^p·c^p)/3)^(1/p) with p ≈ 1.6075.

**WARNING:** The simple formula `m/3*(b²+c²)` from `future_work_11.md`
(§43) gives Ix=212211.70 — **WRONG** (MuJoCo: 180751.03, error 17.4%).
MuJoCo uses an analytical shell inertia formula for ellipsoids that is NOT
a simple coefficient swap from the solid formula (`m/5*(b²+c²)`). Scale
invariance verified: at scales 0.1×, 1.0×, 10.0×, the normalized values
I/(m·scale²) are identical — confirming an analytical formula (not
tessellation). The exact formula requires C source (`user_mesh.cc` →
`SetGeom()`) investigation. Empirical values for spec:

| Semi-axes | Mass | Ix | Iy | Iz |
|-----------|------|----|----|-----|
| (1,2,3) | 48971.93 | 180751.03 | 140683.07 | 81026.34 |

Additional verification data points should be collected during spec writing.

### EGT-9: `shellinertia` rejected on mesh geoms

MuJoCo 3.5.0 **errors** when `shellinertia="true"` is set on a mesh geom:

```
ValueError: Error: for mesh geoms, inertia should be specified in the mesh asset
```

This is a compile-time validation error, not a runtime behavior.

### EGT-10: `<compiler exactmeshinertia>` removed

MuJoCo 3.5.0 **rejects** `exactmeshinertia` on `<compiler>`:

```
XML Error: Schema violation: unrecognized attribute: 'exactmeshinertia'
```

Attribute fully removed from schema.

### EGT-11: Mesh default class inheritance

`<default><mesh inertia="shell"/>` works:

```xml
<default><mesh inertia="shell"/></default>
<asset><mesh name="cube" vertex="..." face="..."/></asset>
```

Produces shell inertia (mass=6000 for unit cube). ✓

### EGT-12: Legacy mode

L-shape mesh with `inertia="legacy"`, density=1000:

```
body_mass[1] = 3000.0
body_inertia[1] = [1833.33, 1500.0, 833.33]
body_ipos[1] = [0.833, 0.833, 0.5]
```

Convex mode on same mesh: mass=3500, I=[2103.17, 1583.33, 1103.17].
Legacy uses absolute tetrahedron volumes (overcounts non-convex regions).

**Caveat:** For this particular L-shape, exact mode with outward-oriented
normals produces mass=3000 (identical to legacy) because all tetrahedra
from the centroid have positive signed volume. Legacy overcounting is only
observable on meshes where the centroid falls outside the solid (e.g., a
C-shape or U-shape). The spec should include a deeply-concave mesh to
demonstrate the legacy vs exact difference.

### EGT-13: Exact mode rejects misoriented meshes

L-shape mesh with **inward-pointing normals** (reversed face winding) and
`inertia="exact"`:

```
ValueError: Error: mesh volume is negative (misoriented triangles): lshape
```

**Caveat:** This error only triggers with reversed face winding. With
properly outward-oriented normals, exact mode succeeds on the L-shape.
Mixed normals produce a different error: "faces of mesh 'lshape' have
inconsistent orientation." The spec must specify the exact winding
condition that triggers each error.

### Codebase Context

| File | What to change | Current state |
|------|---------------|---------------|
| `sim/L0/mjcf/src/types.rs:842-847` | `MjcfMeshDefaults` — add `inertia: Option<MeshInertia>` | Only has `scale`, `maxhullvert` |
| `sim/L0/mjcf/src/types.rs:858-875` | `MjcfMesh` — add `inertia: Option<MeshInertia>` | 6 fields, no inertia mode |
| `sim/L0/mjcf/src/types.rs:1198-1267` | `MjcfGeom` — add `shellinertia: Option<bool>` | No shellinertia field |
| `sim/L0/mjcf/src/types.rs:271-279` | New `MeshInertia` enum near `InertiaFromGeom` | Does not exist |
| `sim/L0/mjcf/src/types.rs:324-327` | `MjcfCompiler.exactmeshinertia` — deprecation decision | Parsed, stored, unused |
| `sim/L0/mjcf/src/parser.rs:484-486` | `exactmeshinertia` parsing | Currently parsed |
| `sim/L0/mjcf/src/parser.rs` (mesh element) | Parse `inertia` on `<mesh>` | Not parsed |
| `sim/L0/mjcf/src/parser.rs` (geom element) | Parse `shellinertia` on `<geom>` | Not parsed |
| `sim/L0/mjcf/src/builder/mesh.rs:19-21` | `MeshProps` type alias | `(f64, Vector3, Matrix3)` — mode-unaware |
| `sim/L0/mjcf/src/builder/mesh.rs:429-542` | `compute_mesh_inertia()` | Exact algorithm only |
| `sim/L0/mjcf/src/builder/geom.rs:302-343` | `compute_geom_mass()` | Volume-based only; shell needs area-based |
| `sim/L0/mjcf/src/builder/geom.rs:359-451` | `compute_geom_inertia()` | Exact mesh + solid primitives only |
| `sim/L0/mjcf/src/builder/geom.rs:290-300` | `geom_effective_com()` | Uses mesh COM from exact props; shell COM differs |
| `sim/L0/mjcf/src/builder/mass.rs:144-225` | `compute_inertia_from_geoms()` | Calls above functions; unchanged but inputs change |
| `sim/L0/core/src/convex_hull.rs` | Spec A's `ConvexHull` — read-only consumer | Complete (Spec A landed) |
| `sim/L0/core/src/mesh.rs` | `TriangleMeshData` — `convex_hull()` accessor | Complete (Spec A landed) |
| `sim/L0/tests/integration/exactmeshinertia.rs` | Extend with shell/convex/legacy tests | 10 tests (AC1-AC10) for exact only |
| `sim/L0/mjcf/src/defaults.rs:731-744` | `apply_to_mesh()` — cascade `inertia` field | Currently cascades `scale`, `maxhullvert` only |
| `sim/L0/mjcf/src/defaults.rs:233-334` | `apply_to_geom()` — cascade `shellinertia` field | Cascades ~30 fields over 101 lines, no `shellinertia` |
| `sim/L0/mjcf/src/defaults.rs:854-893` | `merge_geom_defaults()` — merge `shellinertia` with `.or()` | Merges ~30 fields with `.or()` pattern, no `shellinertia` |
| `sim/L0/mjcf/src/defaults.rs:993-1006` | `merge_mesh_defaults()` — merge `inertia` with `.or()` | Merges `scale`, `maxhullvert` only — **required for EGT-11 inheritance** |

**Match sites that will need updating:**

| Location | Pattern | Risk |
|----------|---------|------|
| `geom.rs:311` | `MjcfGeomType::Mesh =>` in `compute_geom_mass()` | Must dispatch on inertia mode for volume vs area |
| `geom.rs:435` | `MjcfGeomType::Mesh =>` in `compute_geom_inertia()` | Must dispatch on inertia mode for algorithm selection |
| `geom.rs:293` | `if let Some(&(_, mesh_com, _)) = mesh_props` in `geom_effective_com()` | COM depends on inertia mode |
| `mesh.rs:19-21` | `MeshProps` type alias | May need mode info or separate shell/exact variants |
| `MjcfMesh::default()` at types.rs:877-885 | Manual `impl Default` | Must include `inertia: None` |
| `MjcfGeom::default()` at types.rs:1269+ | Manual `impl Default` | Must include `shellinertia: None` |
| `MjcfMeshDefaults::default()` at types.rs:842 | `#[derive(Default)]` | Add `inertia: None` via derive (Option defaults to None) |
| `MjcfGeomDefaults` at defaults.rs:862+ | `merge_geom_defaults()` `.or()` pattern | Must add `shellinertia` to merge chain |
| `mesh.rs:754,779,795,824` | Test `MjcfMesh { .. }` struct literals (4 sites) | Will fail to compile when `inertia` field added |
| `geom.rs:535-559` | Test `MjcfGeom { .. }` struct literal (1 site) | Will fail to compile when `shellinertia` field added |

---

## Criteria

> **Criterion priority:** P1 (MuJoCo Reference Fidelity) is the cardinal
> criterion. Grade P1 first and grade it hardest. If P1 is not A+, do not
> proceed to grading other criteria until it is fixed.

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does — exact function names, field
> names, calling conventions, and edge cases. No hand-waving.

| Grade | Bar |
|-------|-----|
| **A+** | Every MuJoCo function/field/flag cited with source file and exact behavior. Specific citations required: `mjsMesh.inertia` field type (`mjtMeshInertia`), `mjsGeom.typeinertia` field type (`mjtGeomInertia`), `mjtMeshInertia` enum values with ordinals (CONVEX=0, EXACT=1, LEGACY=2, SHELL=3), default value (CONVEX=0). Shell mesh algorithm from `user_mesh.cc` → `ComputeInertia()` — area-weighted per-triangle with divisor 12 (shell) vs 20 (solid volume). Primitive shell formulas from `user_mesh.cc` → `SetGeom()` for all 5 geom types (sphere, box, cylinder, capsule, ellipsoid). Legacy algorithm from `user_mesh.cc` (absolute tetrahedron volumes). Edge cases addressed: (1) exact mode rejects misoriented/negative-volume meshes, (2) `shellinertia` rejected on mesh geoms in MuJoCo 3.5.0, (3) degenerate zero-area mesh fallback, (4) explicit mass override with shell density, (5) `exactmeshinertia` removal from `<compiler>`. Numerical expectations from EGT-1 through EGT-13 included. |
| **A** | MuJoCo behavior described correctly from C source. Minor gaps in edge-case coverage. |
| **B** | Correct at high level, but missing specifics or based on docs rather than C source. |
| **C** | Partially correct. Some MuJoCo behavior misunderstood or invented. |

**P1/P9 boundary:** P1 grades whether the spec GOT the MuJoCo reference
right (factual accuracy). P9 (Inertia Mode Dispatch) grades whether the
dispatch architecture correctly selects and invokes the right algorithm.

### P2. Algorithm Completeness

> Every algorithmic step is specified unambiguously. No "see MuJoCo source"
> or "TBD" gaps. Rust code is line-for-line implementable.

| Grade | Bar |
|-------|-----|
| **A+** | Four distinct algorithms specified in Rust: (1) Shell mesh: area-weighted triangle decomposition with divisor 12, per-triangle area + centroid + second-moment accumulation, COM shift via parallel axis theorem — all formulas written out. (2) Convex mesh: call existing `compute_mesh_inertia()` on `convex_hull.vertices/faces` — exact code path specified. (3) Legacy mesh: absolute tetrahedron volumes (replace `det/6` with `det.abs()/6`) — single-line change from exact algorithm identified. (4) Primitive shell formulas for 5 geom types: sphere (`2/3 mr²`), box (face-decomposition with PAT — NOT `m/6*(b²+c²)`), cylinder (lateral+caps decomposition), capsule (cylinder-shell + hemisphere-shell with PAT from hemisphere COM at r/2 above equator — NOT from sphere center), ellipsoid (**requires C source reverse-engineering** — simple `m/3*(b²+c²)` is WRONG by 17%; MuJoCo uses an unknown scale-invariant analytical formula; empirical values from EGT-8 must be matched). Each formula matches MuJoCo's `SetGeom()`. |
| **A** | Algorithms are complete. One or two details left implicit. |
| **B** | Structure clear but some steps hand-waved or deviate from MuJoCo. |
| **C** | Skeleton only. |

### P3. Convention Awareness

> Spec explicitly addresses CortenForge conventions where they differ from
> MuJoCo.

| Grade | Bar |
|-------|-----|
| **A+** | Convention table mapping: `mjtMeshInertia` → `MeshInertia` enum (variant names, ordinals, default). `mjtGeomInertia` → `shellinertia: Option<bool>` (boolean attribute maps to volume/shell enum). XML attribute `shellinertia` → Rust field naming convention decision documented (snake_case `shell_inertia` vs verbatim `shellinertia`). `mjsMesh.inertia` → `MjcfMesh.inertia: Option<MeshInertia>`. `MeshProps` type alias update strategy. `compute_mesh_inertia()` return value semantics (`volume` becomes `area` for shell mode — explain how callers handle this). Size conventions: MuJoCo `size` is half-extents for box, half-height for cylinder/capsule (consistent with existing CortenForge code). Density interpretation: `mass = density * volume` for solid, `mass = density * area` for shell. |
| **A** | Major conventions documented. Minor field-name mappings left to implementer. |
| **B** | Some conventions noted, others not. |
| **C** | MuJoCo code pasted without adaptation. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable. Contains concrete values.

| Grade | Bar |
|-------|-----|
| **A+** | Every AC has three-part structure: (1) concrete MJCF model, (2) exact expected value with tolerance, (3) field to check. At least one AC per inertia mode verified against MuJoCo 3.5.0 output. Specific required ACs: unit cube shell (mass=6000, I=1666.67), sphere shell (mass=12566.37, I=8377.58), box shell (mass=88000, I=[541333.33, 421333.33, 242666.67]), cylinder shell (mass=31415.93, I=[72780.23, 72780.23, 28274.33]), capsule shell (mass=37699.11, I=[129852.50, 129852.50, 33510.32]), ellipsoid shell (mass=48971.93, I=[180751.03, 140683.07, 81026.34]), convex mode on non-convex mesh (mass=3500 for L-shape), legacy mode (mass=3000 for L-shape), exact mode on misoriented mesh (error), explicit mass override with shell, `shellinertia` rejected on mesh geom, default class inheritance, default mode is `Convex`. Code-review ACs labeled separately. |
| **A** | ACs are testable. Some lack exact MuJoCo-verified values. |
| **B** | ACs directionally correct but vague. |
| **C** | ACs are aspirational statements. |

### P5. Test Plan Coverage

> Tests cover happy path, edge cases, regressions, and interactions.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present. Edge case inventory covers: (1) zero-area degenerate mesh with shell mode, (2) misoriented mesh with exact mode (negative volume error), (3) explicit mass override with each mode, (4) `shellinertia` on mesh geom rejection, (5) `shellinertia` on each of 5 primitive types, (6) convex mode on mesh that's already convex (== exact), (7) default class inheritance for both `<mesh inertia>` and `<geom shellinertia>`, (8) multi-geom body mixing shell and solid geoms. At least one MuJoCo conformance test per inertia mode. Existing `exactmeshinertia.rs` AC1-AC10 tests verified as non-regressing. |
| **A** | Good coverage. Minor edge-case gaps. |
| **B** | Happy path covered. Edge cases sparse. |
| **C** | Minimal test plan. |

### P6. Dependency Clarity

> Prerequisites, ordering constraints, and interactions with other specs.

| Grade | Bar |
|-------|-----|
| **A+** | Spec A (§65 convex hull) cited as hard prerequisite for Convex mode — commit hash of Spec A completion stated. Execution order unambiguous: S1 (types) → S2 (parsing) → S3 (shell algorithm) → S4 (convex/legacy) → S5 (primitive shell) → S6 (dispatch integration). Each section states what it requires from prior sections. Cross-spec interaction with Spec A's `ConvexHull` API explicitly documented. `exactmeshinertia` deprecation sequencing clear (which step removes/updates it). |
| **A** | Order is clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Spec identifies every file touched and every behavior that changes.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list with per-file change description. Behavioral changes: (1) mesh geoms now dispatch on inertia mode (behavior change for `MeshInertia::Convex` default vs current `Exact` — verify numerically identical for watertight convex meshes). (2) `shellinertia` attribute accepted on `<geom>`. (3) `inertia` attribute accepted on `<mesh>`. Existing test impact: all 10 `exactmeshinertia.rs` tests must still pass (they use default mode, which is now Convex — numerically identical for the cube mesh). Data staleness: `MjcfMesh` default impl, `MjcfGeom` default impl, `MjcfMeshDefaults` derive, `MjcfGeomDefaults` merge pattern in `merge_geom_defaults()`, `merge_mesh_defaults()` merge pattern. |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. |
| **C** | No blast-radius analysis. |

### P8. Internal Consistency

> No contradictions within the spec. Shared concepts use identical terminology.

| Grade | Bar |
|-------|-----|
| **A+** | Terminology uniform: `MeshInertia` enum used consistently (not mixed with `mjtMeshInertia` or `inertia_mode`). File paths match between Specification and Files Affected. AC numbers match between AC section and Traceability Matrix. Edge case lists consistent across MuJoCo Reference and Test Plan. Shell formula divisor "12" stated consistently everywhere. Enum variant ordering matches MuJoCo C enum (Convex=0, Exact=1, Legacy=2, Shell=3). |
| **A** | Consistent. One or two minor terminology inconsistencies. |
| **B** | Some sections use different names for same concept. |
| **C** | Contradictions between sections. |

### P9. Inertia Mode Dispatch Architecture

> The dispatch logic correctly selects the right algorithm based on mesh
> inertia mode and geom attributes, handling all mode × geom-type
> combinations.

| Grade | Bar |
|-------|-----|
| **A+** | Dispatch matrix explicitly stated: (mode × geom-type) → algorithm. Covers: `Exact` → `compute_mesh_inertia()`, `Shell` → `compute_mesh_inertia_shell()`, `Convex` → `compute_mesh_inertia(convex_hull)`, `Legacy` → `compute_mesh_inertia_legacy()`. Primitive shell dispatch: `shellinertia=true` × {sphere, box, capsule, cylinder, ellipsoid} → shell formula. Interaction: mesh mode vs geom shellinertia for mesh geoms (reject shellinertia, use mesh mode). Resolution order when both are set. Mass computation changes per mode (volume-based vs area-based). COM computation changes per mode. `MeshProps` type generalization to support both volume and area "measure" fields. |
| **A** | Dispatch is clear. Minor interaction gaps. |
| **B** | Basic dispatch defined but interactions unclear. |
| **C** | No dispatch architecture. |

### P10. Backward Compatibility

> Changes preserve existing behavior for users who don't opt into new
> features.

| Grade | Bar |
|-------|-----|
| **A+** | Default behavior analysis: meshes without `inertia` attribute use `Convex` mode (MuJoCo default). For watertight convex meshes (most common case), `Convex` == `Exact` numerically. For non-convex meshes, `Convex` uses hull — this IS a behavior change from current CortenForge (which uses Exact). Spec explicitly addresses: (a) whether to match MuJoCo 3.5.0 default (Convex) or maintain CortenForge's current default (Exact), (b) impact on existing tests, (c) `exactmeshinertia` attribute handling — keep for parse compatibility or remove matching MuJoCo 3.5.0. Migration path documented for any breaking change. |
| **A** | Backward compat analyzed. Minor migration gaps. |
| **B** | Some compat concerns noted. |
| **C** | No backward compat discussion. |

---

## Rubric Self-Audit

### Self-audit checklist

- [x] **Specificity:** Every A+ bar names specific MuJoCo functions
      (`ComputeInertia`, `SetGeom`, `mjtMeshInertia`), enum values with
      ordinals, and concrete numerical expectations from EGT-1 through
      EGT-13. Two independent reviewers could assign consistent grades.

- [x] **Non-overlap:** P1 grades factual accuracy of MuJoCo reference. P2
      grades algorithm completeness (formulas written in Rust). P9 grades
      dispatch architecture (mode selection logic). P10 grades backward
      compat impact. Overlap is minimal and managed: P1/P9 share algorithm
      description (factual vs architectural), P1/P4 share numerical values
      (reference accuracy vs test rigor), P2/P9 share dispatch logic
      (formula completeness vs selection architecture). P1/P9 boundary
      stated explicitly.

- [x] **Completeness:** 10 criteria cover: conformance (P1), algorithms (P2),
      conventions (P3), ACs (P4), tests (P5), dependencies (P6), blast
      radius (P7), consistency (P8), dispatch architecture (P9), backward
      compat (P10). No meaningful gap survives all 10.

- [x] **Gradeability:** P1→MuJoCo Reference section. P2→Specification
      (S1..SN). P3→Convention Notes. P4→Acceptance Criteria. P5→Test Plan.
      P6→Prerequisites/Execution Order. P7→Risk & Blast Radius. P8→cross-
      cutting. P9→Specification dispatch logic. P10→Behavioral Changes.

- [x] **Conformance primacy:** P1 requires specific C function citations,
      enum ordinals, and EGT values. P4 requires MuJoCo-verified expected
      values. P5 requires conformance tests. P9 requires dispatch matching
      MuJoCo's mode selection. An A+ spec cannot diverge from MuJoCo.

- [x] **Empirical grounding:** EGT-1 through EGT-13 filled with MuJoCo
      3.5.0 Python output. Codebase context table identifies every file,
      function, and match site. Criterion bars verified empirically; enum
      ordinals confirmed via both `mjspec.h` header and behavioral tests.

- [x] **Stress-tested (Rev 2):** Two formula errors from `future_work_11.md`
      caught (box shell R8, ellipsoid shell R9). Three missing codebase
      match sites added (R10-R12). Capsule shell PAT correction documented.

- [x] **Double stress-tested (Rev 3):** All 13 EGT values independently
      re-verified against MuJoCo 3.5.0. Five additional gaps found and
      fixed: `merge_mesh_defaults()` missing (R13), EGT-12/13 L-shape
      caveats (R14-R15), P4 primitive coverage gap (R16), `shellinertia`
      naming convention (R17). Self-audit non-overlap and empirical
      grounding claims tightened.

### Criterion → Spec section mapping

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P1 | MuJoCo Reference, Key Behaviors table, Convention Notes |
| P2 | Specification (S1–S6), algorithm code blocks |
| P3 | Convention Notes, Specification code |
| P4 | Acceptance Criteria |
| P5 | Test Plan, AC→Test Traceability Matrix, Edge Case Inventory |
| P6 | Prerequisites, Execution Order |
| P7 | Risk & Blast Radius (Behavioral Changes, Files Affected, Existing Test Impact) |
| P8 | *Cross-cutting — all sections checked for mutual consistency* |
| P9 | Specification dispatch logic, mode × geom-type matrix |
| P10 | Behavioral Changes, migration paths, default value analysis |

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
| P9. Inertia Mode Dispatch | | |
| P10. Backward Compatibility | | |

**Overall: — (Rev 3 — double stress-tested)**

---

## Gap Log

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| R1 | P1 | Umbrella claimed default = `exact`; MuJoCo 3.5.0 default = `convex` (enum value 0) | EGT-1, mjspec.h line 67 | Corrected in Scope Adjustment; P1 A+ bar requires correct default | Rubric Rev 1 |
| R2 | P1 | `exactmeshinertia` removed from MuJoCo 3.5.0 — not just deprecated | EGT-10 empirical test | Added to Scope Adjustment; P10 backward compat criterion addresses handling | Rubric Rev 1 |
| R3 | P1 | `shellinertia` rejected on mesh geoms (MuJoCo 3.5.0 error) — umbrella implied it works as backward compat for mesh inertia | EGT-9 empirical test | Corrected in Scope Adjustment; P9 dispatch criterion requires rejection | Rubric Rev 1 |
| R4 | P10 | Default change: CortenForge currently always uses Exact; MuJoCo 3.5.0 defaults to Convex | EGT-1 + codebase context | Added P10 criterion for backward compat analysis | Rubric Rev 1 |
| R5 | P3 | `mjtGeomInertia` enum (VOLUME=0, SHELL=1) maps to `shellinertia` boolean — enum vs bool convention mismatch | mjspec.h line 60-63 | P3 A+ bar requires mapping documentation | Rubric Rev 1 |
| R6 | P1 | Ellipsoid shell uses Thomsen surface area approximation — not closed form | EGT-8 empirical test (values don't match simple 4π/3 formulas) | P2 A+ bar requires specifying Thomsen formula | Rubric Rev 1 |
| R7 | P9 | `<default><mesh inertia="shell"/>` inheritance — not mentioned in umbrella | EGT-11 empirical test | Added to scope; P9 dispatch must handle defaults merging | Rubric Rev 1 |
| R8 | P2 | Box shell formula `m/6*(b²+c²)` from `future_work_11.md` (§43) is **WRONG** — produces 762666.67 vs MuJoCo's 541333.33. Correct algorithm: face-by-face decomposition (6 rectangular plates with PAT). | Stress test empirical verification | P2 A+ bar updated; EGT-5 WARNING added with correct derivation | Rubric Rev 2 |
| R9 | P2 | Ellipsoid shell formula `m/3*(b²+c²)` from `future_work_11.md` (§43) is **WRONG** — produces 212211.70 vs MuJoCo's 180751.03 (17.4% error). MuJoCo uses unknown scale-invariant analytical formula. | Stress test empirical verification + scale invariance test | P2 A+ bar flagged as requiring C source investigation; EGT-8 WARNING added | Rubric Rev 2 |
| R10 | P7 | `defaults.rs` missing from rubric codebase context — 3 match sites: `apply_to_mesh()` (line 731), `apply_to_geom()` (line 233), `merge_geom_defaults()` (line 854) | Stress test codebase audit | Added to Codebase Context table | Rubric Rev 2 |
| R11 | P9 | `MjcfGeomDefaults` merge pattern needs `shellinertia` in `.or()` chain at defaults.rs:862+ | Stress test codebase audit | Added to Match Sites table | Rubric Rev 2 |
| R12 | P7 | Test struct literals will break on field addition: 4 sites in mesh.rs (lines 754, 779, 795, 824) and 1 in geom.rs (lines 535-559) | Stress test codebase audit | Added to Match Sites table | Rubric Rev 2 |
| R13 | P7 | `merge_mesh_defaults()` at defaults.rs:993-1006 missing from codebase context — required for EGT-11 `<default><mesh inertia>` inheritance | Stress test round 2 codebase audit | Added to Codebase Context table | Rubric Rev 3 |
| R14 | P1 | EGT-13 only triggers with reversed face winding; outward-oriented L-shape succeeds in exact mode. Mixed normals produce a different error ("inconsistent orientation"). Rubric framing was misleading. | Stress test round 2 empirical | EGT-13 updated with winding caveat and multiple error conditions | Rubric Rev 3 |
| R15 | P1 | EGT-12 legacy overcounting NOT observable on L-shape (centroid inside solid → all tetrahedra positive). Need deeper concavity (C-shape) to demonstrate. | Stress test round 2 empirical | EGT-12 caveat added; spec must include deeply-concave mesh | Rubric Rev 3 |
| R16 | P4 | P4 required ACs for only 2 of 5 primitive shell types (sphere, box). Cylinder, capsule, ellipsoid had EGT data but were not required by P4 A+ bar. | Stress test round 2 consistency audit | P4 A+ bar expanded to require all 5 primitive types + default mode + exact error | Rubric Rev 3 |
| R17 | P3 | `shellinertia` used as Rust field name without acknowledging snake_case convention (`shell_inertia`). Naming decision undocumented. | Stress test round 2 consistency audit | P3 A+ bar updated to require naming convention decision | Rubric Rev 3 |
