# Mesh Inertia Modes — Spec B

**Status:** Draft
**Phase:** Roadmap Phase 9 — Collision Completeness
**Effort:** M
**MuJoCo ref:** `ComputeInertia()` in `user_mesh.cc`, `SetGeom()` in `user_mesh.cc`
**MuJoCo version:** 3.5.0
**Test baseline:** 1,900+ sim domain tests
**Prerequisites:**
- Spec A (§65 convex hull) complete — commit `8a6b67b`

**Independence:** This spec depends on Spec A per the umbrella dependency graph.
Shared files: `sim/L0/mjcf/src/types.rs` (Spec A added `maxhullvert` fields;
Spec B adds `inertia` and `shellinertia` fields — no conflict),
`sim/L0/core/src/mesh.rs` (Spec A added `convex_hull`; Spec B reads it — no
conflict), `sim/L0/mjcf/src/builder/mesh.rs` (Spec A added hull computation;
Spec B adds shell/legacy inertia algorithms — different code sections).

> **Conformance mandate:** This spec exists to make CortenForge produce
> numerically identical results to MuJoCo for the feature described below.
> Every algorithm, every acceptance criterion, and every test is in service
> of this goal. The MuJoCo C source code is the single source of truth —
> not the MuJoCo documentation, not intuition about "what should happen,"
> not a Rust-idiomatic reinterpretation. When in doubt, read the C source
> and match its behavior exactly.

---

## Scope Adjustment

| Umbrella claim | MuJoCo 3.5.0 reality | Action |
|----------------|----------------------|--------|
| Default mesh inertia is `exact` | Default is `convex` (`mjMESH_INERTIA_CONVEX = 0`). For convex meshes, `convex` == `exact` numerically. | **Correct:** default = `Convex` |
| `<compiler exactmeshinertia>` exists (deprecated) | **Removed** in MuJoCo 3.5.0 — parser rejects as "unrecognized attribute" | **Keep parsing + warn** (backward compat); no behavioral effect |
| `shellinertia` on `<geom>` works for mesh geoms | MuJoCo **rejects** `shellinertia` on mesh geoms: "for mesh geoms, inertia should be specified in the mesh asset" | **In scope:** reject on mesh geoms |
| Primitive shell formulas: sphere, box, capsule, cylinder, ellipsoid | All 5 produce shell inertia via `shellinertia="true"` | **In scope:** all 5 primitives |
| `<default><mesh inertia="shell"/>` inheritance | Supported — mesh defaults inherit `inertia` attribute | **In scope:** add to `MjcfMeshDefaults` cascade |
| Legacy mode: backward compat algorithm | `inertia="legacy"` uses absolute tetrahedron volumes (overcounting non-convex regions) | **In scope** |

**Final scope:**

1. `MeshInertia` enum (`Convex`, `Exact`, `Legacy`, `Shell`) on `MjcfMesh`
2. `inertia` attribute parsing on `<mesh>` element (default: `Convex`)
3. `inertia` inheritance via `<default><mesh/>`
4. Shell mesh inertia algorithm (area-weighted surface distribution, per-triangle)
5. Convex mesh inertia (exact algorithm on Spec A's convex hull)
6. Legacy mesh inertia (absolute tetrahedron volumes)
7. `shellinertia` attribute on `<geom>` for primitive shell inertia
8. Primitive shell inertia formulas (sphere, box, capsule, cylinder, ellipsoid)
9. `shellinertia` rejected on mesh geoms
10. Ellipsoid solid volume bug fix (missing from `compute_geom_mass()`)
11. `exactmeshinertia` deprecation handling (keep parsing, warn, no effect)

---

## Problem Statement

**Conformance gap.** MuJoCo implements four mesh inertia modes via the
`mjtMeshInertia` enum (`Convex`, `Exact`, `Legacy`, `Shell`) in
`user_mesh.cc` → `ComputeInertia()`. CortenForge only implements `Exact`
(signed tetrahedron decomposition). The default mode in MuJoCo 3.5.0 is
`Convex` (enum value 0), not `Exact` — meaning CortenForge's current
default behavior differs from MuJoCo for non-convex meshes.

Additionally, MuJoCo supports `shellinertia` on primitive geoms (`<geom>`),
switching mass computation from volume-based (`density × volume`) to
area-based (`density × surface_area`) with shell inertia tensors. This is
not implemented in CortenForge.

Pre-existing bug: `compute_geom_mass()` in `builder/geom.rs` has no
`Ellipsoid` arm — it falls to `_ => 0.001`, producing incorrect mass for
density-based ellipsoid geoms. This is fixed as part of this spec since
correct mass computation is a prerequisite for shell inertia.

---

## MuJoCo Reference

### Enum and struct layout

**`mjtMeshInertia`** (`mjspec.h:67`):
```c
typedef enum mjtMeshInertia_ {
  mjMESH_INERTIA_CONVEX = 0,  // default (zero-initialized)
  mjMESH_INERTIA_EXACT  = 1,
  mjMESH_INERTIA_LEGACY = 2,
  mjMESH_INERTIA_SHELL  = 3,
} mjtMeshInertia;
```

**`mjtGeomInertia`** (`mjspec.h:60–63`):
```c
typedef enum mjtGeomInertia_ {
  mjGEOM_INERTIA_VOLUME = 0,  // default
  mjGEOM_INERTIA_SHELL  = 1,
} mjtGeomInertia;
```

**`mjsMesh.inertia`**: type `mjtMeshInertia`, XML attribute `inertia` on
`<mesh>`, keywords `"convex"`, `"exact"`, `"legacy"`, `"shell"`.

**`mjsGeom.typeinertia`**: type `mjtGeomInertia`, XML attribute
`shellinertia` on `<geom>`, boolean `"true"/"false"`. Maps: `false` →
`mjGEOM_INERTIA_VOLUME`, `true` → `mjGEOM_INERTIA_SHELL`.

### Shell mesh inertia algorithm

**Source:** `user_mesh.cc` → `ComputeInertia()`, shell path.

For a triangle mesh with vertices {vᵢ} and triangles {T}, shell mode
computes mass properties by treating the mesh surface as a thin shell with
uniform surface density σ = density (kg/m²):

1. **Per-triangle area:** `A_t = ½ ‖(b−a) × (c−a)‖`
2. **Total surface area:** `SA = Σ A_t`
3. **Mass:** `m = density × SA`
4. **COM:** `com = (1/SA) × Σ A_t × (a+b+c)/3`
5. **Second-moment integrals (surface distribution):**
   ```
   ∫_T xᵢ² dA = (A_t/6) × (aᵢ² + bᵢ² + cᵢ² + aᵢbᵢ + aᵢcᵢ + bᵢcᵢ)
   ∫_T xᵢxⱼ dA = (A_t/12) × (2aᵢaⱼ + 2bᵢbⱼ + 2cᵢcⱼ
                    + aᵢbⱼ + aⱼbᵢ + aᵢcⱼ + aⱼcᵢ + bᵢcⱼ + bⱼcᵢ)
   ```

   Compare to volume mode:
   - Volume: `∫_tet xᵢ² dV = (det/60) × (...)`, cross: `det/120`
   - Shell: `∫_T xᵢ² dA = (A_t/6) × (...)`, cross: `A_t/12`

6. **Inertia tensor at origin:** `I_origin[i,i] = Σ(other two) − Σ(cross)`
   Same structure as volume mode but using surface integrals.

7. **Shift to COM via parallel axis theorem:** `I_com = I_origin − m(d·d I₃ − d⊗d)`

### Legacy mesh inertia algorithm

**Source:** `user_mesh.cc` → `ComputeInertia()`, legacy path.

Identical to exact mode except tetrahedron volumes use `|det|/6` instead of
`det/6` (absolute value). For convex meshes where all tetrahedra from the
centroid have positive signed volume, legacy == exact. For non-convex meshes
where the centroid falls outside the solid (e.g., C-shape), legacy
overcounts by treating negative-volume regions as positive. This is the
pre-3.0 MuJoCo behavior.

### Convex mesh inertia

**Source:** `user_mesh.cc` → `ComputeInertia()`, convex path.

Runs the exact (signed tetrahedron) algorithm on the mesh's convex hull
geometry instead of the original mesh. Equivalent to
`compute_mesh_inertia(convex_hull)`. For convex meshes, `Convex == Exact`
numerically (the hull IS the original mesh).

### Primitive shell inertia

**Source:** `user_mesh.cc` → `SetGeom()`, shell path for each primitive type.

When `shellinertia="true"` on `<geom>`, mass = density × surface_area
(instead of density × volume). Inertia tensors use shell formulas:

**Sphere** (radius r):
- SA = 4πr², mass = ρ × 4πr²
- I = 2/3 × m × r² (all three axes)

**Box** (half-extents a, b, c):
- SA = 8(ab + ac + bc), mass = ρ × SA
- Face decomposition: 6 rectangular plates, each a thin plate with its own
  moment of inertia about its center, shifted to the box center via PAT.
- For ±x faces (area 4bc, mass mf = ρ × 4bc):
  - Ix_face = mf × (b² + c²)/3
  - Iy_face = mf × (a² + c²/3)
  - Iz_face = mf × (a² + b²/3)
- For ±y faces (area 4ac, mass mf = ρ × 4ac):
  - Ix_face = mf × (b² + c²/3)
  - Iy_face = mf × (a² + c²)/3
  - Iz_face = mf × (b² + a²/3)
- For ±z faces (area 4ab, mass mf = ρ × 4ab):
  - Ix_face = mf × (b²/3 + c²)
  - Iy_face = mf × (a²/3 + c²)
  - Iz_face = mf × (a² + b²)/3
- Total: sum all 6 faces (2 per axis pair).

**WARNING:** The formula `m/6(b²+c²)` from `future_work_11.md` is
**WRONG** — it produces 762,666.67 vs MuJoCo's 541,333.33 for box
(1,2,3). The correct algorithm is face decomposition as above.

**Cylinder** (radius r, half-height h_half, full height h = 2×h_half):
- SA = 2πrh + 2πr², mass = ρ × SA
- Decompose into lateral shell + 2 end caps:
  - Lateral: m_lat = ρ × 2πrh
    - Ix = m_lat × (r²/2 + h²/12), Iz = m_lat × r²
  - Cap (each): m_cap = ρ × πr²
    - Ix_own = m_cap × r²/4, Iz_own = m_cap × r²/2
    - PAT: Ix_shifted = Ix_own + m_cap × (h/2)²

**Capsule** (radius r, half-height h_half, full height h = 2×h_half):
- SA = 2πrh + 4πr², mass = ρ × SA
- Decompose into lateral shell (open cylinder) + 2 hemisphere shells:
  - Lateral: m_lat = ρ × 2πrh
    - Ix = m_lat × (r²/2 + h²/12), Iz = m_lat × r²
  - Hemisphere (each): m_hemi = ρ × 2πr²
    - COM at distance r/2 from equator (NOT at sphere center)
    - Ix_own = m_hemi × 5r²/12 (about hemisphere COM)
    - Iz_own = 2/3 × m_hemi × r²
    - PAT distance: d = h/2 + r/2
    - Ix_shifted = Ix_own + m_hemi × d²

**WARNING:** Naive PAT from sphere center (d = h/2) gives Ix=104,719.76 —
**WRONG**. Must use hemisphere COM offset (d = h/2 + r/2) which gives
Ix=129,852.50 matching MuJoCo.

**Ellipsoid** (semi-axes a, b, c):
- SA via Knud Thomsen approximation:
  `SA ≈ 4π × ((aᵖbᵖ + aᵖcᵖ + bᵖcᵖ)/3)^(1/p)`, p = 1.6075
- mass = ρ × SA
- Inertia: numerical quadrature on the parametric ellipsoid surface.
  The parametric surface r(θ,φ) = (a sinθ cosφ, b sinθ sinφ, c cosθ),
  θ ∈ [0,π], φ ∈ [0,2π]. The surface element:
  `dS = sinθ × √(c²sin²θ(b²cos²φ + a²sin²φ) + a²b²cos²θ) dθ dφ`
  The second moments:
  `∫∫ xᵢ² dS = ∫₀^π ∫₀^{2π} xᵢ²(θ,φ) × (dS/dθdφ) dθ dφ`
  Compute via Gauss-Legendre quadrature with N_θ=64, N_φ=128 points.
  Cross moments ∫xᵢxⱼ dS = 0 by symmetry for axis-aligned ellipsoids.

**Note:** The exact MuJoCo algorithm for ellipsoid shell inertia is not
confirmed from C source. The empirical values (EGT-8) are the conformance
target. The numerical quadrature approach converges to the analytical
integral and should match MuJoCo within <0.2% (Thomsen SA approximation
accuracy). If values differ by more than 1%, investigate MuJoCo's exact
approach during implementation.

### Validation rules

1. **`shellinertia` on mesh geom:** MuJoCo 3.5.0 rejects with error:
   `"for mesh geoms, inertia should be specified in the mesh asset"`

2. **Negative volume in exact mode:** MuJoCo rejects with:
   `"mesh volume is negative (misoriented triangles): {name}"`
   Only triggers with fully reversed face winding. Mixed normals produce:
   `"faces of mesh '{name}' have inconsistent orientation"`

3. **`exactmeshinertia` removed:** MuJoCo 3.5.0 rejects as unrecognized
   attribute. CortenForge keeps parsing for backward compat + emits warning.

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| Default mesh inertia mode | `Convex` (enum 0) — `user_mesh.cc` | Always `Exact` — no mode concept |
| Shell mesh inertia | `ComputeInertia()` shell path: area-weighted surface triangles, divisor 6/12 | **Not implemented** |
| Legacy mesh inertia | `ComputeInertia()` legacy path: `\|det\|/6` | **Not implemented** |
| Convex mesh inertia | Exact on convex hull via `ComputeInertia()` | **Not implemented** (hull exists from Spec A but not used for inertia) |
| Primitive shell formulas | `SetGeom()` shell path: 5 geom types | **Not implemented** |
| `shellinertia` on `<geom>` | Parsed; rejected on mesh geoms | **Not parsed** |
| `inertia` on `<mesh>` | Parsed; 4-value enum | **Not parsed** |
| `exactmeshinertia` on `<compiler>` | **Removed** from schema | Parsed and stored (no behavioral effect) |
| Ellipsoid volume in mass | `4/3 π a b c` | **Missing** — falls to `_ => 0.001` in `compute_geom_mass()` |

### Convention Notes

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| `mjtMeshInertia` enum | C enum: CONVEX=0, EXACT=1, LEGACY=2, SHELL=3 | `MeshInertia` Rust enum: `Convex`, `Exact`, `Legacy`, `Shell` | Match ordinal values for serialization; default = `Convex` |
| `mjtGeomInertia` enum | C enum: VOLUME=0, SHELL=1 | `shellinertia: Option<bool>` on `MjcfGeom` | `None` = not set (inherit default), `Some(false)` = volume, `Some(true)` = shell |
| `shellinertia` attribute name | XML attribute `shellinertia` | Rust field `shellinertia` (verbatim, not snake_case `shell_inertia`) | Match MuJoCo XML attribute name for parser clarity |
| `size` for box | Half-extents (a, b, c) | Half-extents (same) | Direct port — no translation needed |
| `size` for cylinder/capsule | `[radius, half_height]` | `[radius, half_height]` (same) | Direct port — `h = 2 × size[1]` for full height |
| Shell mass computation | `mass = density × surface_area` | N/A (not implemented) | For shell mode, `compute_geom_mass()` returns `density × SA` |
| `MeshProps` tuple | `(volume, com, inertia)` for volume; `(area, com, inertia)` for shell | `(f64, Vector3, Matrix3)` — mode-unaware | First element is "measure": volume for Exact/Convex/Legacy, area for Shell. Callers use `.abs()` which works for both. |

---

## Architecture Decisions

### AD-1: Ellipsoid shell inertia method

**Problem:** Ellipsoid shell inertia has no simple closed-form formula
(unlike sphere, box, cylinder, capsule). The `m/3(b²+c²)` formula is
**WRONG** (17% error for semi-axes 1,2,3). MuJoCo uses an unknown
algorithm that is scale-invariant and analytical.

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | Numerical quadrature on parametric surface | Converges to exact integral; scale-invariant; no temporary mesh | May differ from MuJoCo's exact algorithm by SA approximation tolerance (~0.2%) |
| 2 | Tessellate ellipsoid + mesh shell algorithm | Reuses mesh shell code; may match MuJoCo if MuJoCo tessellates | Need to match MuJoCo's tessellation density exactly; extra code |
| 3 | Hardcode empirical values per test case | Matches MuJoCo exactly | Not generalizable; absurd |

**Chosen:** Option 1 — numerical quadrature. The implementation computes
SA via Thomsen approximation (for mass) and inertia moments via
Gauss-Legendre quadrature on the parametric ellipsoid surface. AC tolerance
is 1% relative error vs MuJoCo values to accommodate the SA approximation
difference. If implementation produces >1% error, investigate MuJoCo's
exact algorithm.

### AD-2: Default mode behavioral change

**Problem:** CortenForge currently always uses `Exact` mode (the only
mode). MuJoCo's default is `Convex`. Switching the default changes behavior
for non-convex meshes.

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | Match MuJoCo default (`Convex`) | Full conformance | Behavior change for non-convex meshes without `inertia` attribute |
| 2 | Keep `Exact` as default | No behavior change | Non-conformant with MuJoCo |

**Chosen:** Option 1 — match MuJoCo. For convex meshes (most common),
`Convex == Exact` numerically (zero impact). For non-convex meshes, the
behavioral change moves toward MuJoCo conformance. Existing tests use
convex meshes (cubes, tetrahedra) and will not be affected.

---

## Specification

### S1. MeshInertia enum and type additions

**File:** `sim/L0/mjcf/src/types.rs`
**MuJoCo equivalent:** `mjtMeshInertia` in `mjspec.h:67`
**Design decision:** Match MuJoCo enum ordinals. Use `Option<MeshInertia>`
on `MjcfMesh` (None = not specified in XML, resolved to `Convex` at build
time via default cascade).

Add near `InertiaFromGeom` (around line 270):

```rust
/// Mesh inertia computation mode from `<mesh inertia="..."/>`.
///
/// MuJoCo ref: `mjtMeshInertia` in `mjspec.h`.
/// Ordinals match MuJoCo: Convex=0, Exact=1, Legacy=2, Shell=3.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum MeshInertia {
    /// Compute exact (volumetric) inertia on the convex hull.
    /// Default mode (MuJoCo enum value 0).
    Convex,
    /// Compute exact (volumetric) inertia on the original mesh.
    /// Requires watertight, outward-oriented mesh.
    Exact,
    /// Legacy algorithm: absolute tetrahedron volumes (overcounts
    /// non-convex regions). Pre-MuJoCo 3.0 behavior.
    Legacy,
    /// Surface-area-weighted shell inertia. mass = density × total_area.
    Shell,
}

impl Default for MeshInertia {
    fn default() -> Self {
        Self::Convex // MuJoCo default: enum value 0
    }
}
```

Add `inertia` field to `MjcfMesh` (after `maxhullvert`):

```rust
pub struct MjcfMesh {
    // ... existing fields ...
    pub maxhullvert: Option<usize>,
    /// Mesh inertia computation mode. `None` = not specified in XML
    /// (defaults to `Convex` at build time).
    pub inertia: Option<MeshInertia>,
}
```

Update `MjcfMesh::default()` to include `inertia: None`.

Add `inertia` field to `MjcfMeshDefaults`:

```rust
pub struct MjcfMeshDefaults {
    pub scale: Option<Vector3<f64>>,
    pub maxhullvert: Option<usize>,
    /// Default mesh inertia mode from `<default><mesh inertia="..."/>`.
    pub inertia: Option<MeshInertia>,
}
```

Add `shellinertia` field to `MjcfGeom` (after `user`):

```rust
pub struct MjcfGeom {
    // ... existing fields ...
    pub user: Vec<f64>,
    /// Shell inertia for primitive geoms. `None` = not specified (volume).
    /// `Some(true)` = shell inertia. Rejected on mesh geoms.
    /// MuJoCo: `shellinertia` attribute, maps to `mjtGeomInertia`.
    pub shellinertia: Option<bool>,
}
```

Update `MjcfGeom::default()` to include `shellinertia: None`.

Add `shellinertia` field to `MjcfGeomDefaults` (after `user`):

```rust
pub struct MjcfGeomDefaults {
    // ... existing fields ...
    pub user: Option<Vec<f64>>,
    /// Default shell inertia.
    pub shellinertia: Option<bool>,
}
```

### S2. Parser changes

**File:** `sim/L0/mjcf/src/parser.rs`
**MuJoCo equivalent:** MJCF XML schema for `<mesh>` and `<geom>` elements
**Design decision:** Parse `inertia` as keyword string → `MeshInertia`
enum. Parse `shellinertia` as boolean string → `Option<bool>`.

In `parse_mesh_attrs()` (after `maxhullvert` parsing, ~line 1413):

```rust
// Parse mesh inertia mode
if let Some(inertia_str) = get_attribute_opt(e, "inertia") {
    mesh.inertia = Some(match inertia_str.as_str() {
        "convex" => MeshInertia::Convex,
        "exact" => MeshInertia::Exact,
        "legacy" => MeshInertia::Legacy,
        "shell" => MeshInertia::Shell,
        other => {
            return Err(MjcfError::XmlParse(format!(
                "mesh '{}': invalid inertia mode '{}' \
                 (expected convex, exact, legacy, or shell)",
                mesh.name, other
            )));
        }
    });
}
```

In `parse_mesh_defaults()` (after `maxhullvert`, ~line 1044):

```rust
if let Some(inertia_str) = get_attribute_opt(e, "inertia") {
    defaults.inertia = Some(match inertia_str.as_str() {
        "convex" => MeshInertia::Convex,
        "exact" => MeshInertia::Exact,
        "legacy" => MeshInertia::Legacy,
        "shell" => MeshInertia::Shell,
        other => {
            return Err(MjcfError::XmlParse(format!(
                "mesh default: invalid inertia mode '{}'", other
            )));
        }
    });
}
```

In geom parsing (after `user` parsing, ~line 1940):

```rust
// Parse shellinertia (boolean attribute)
if let Some(si) = get_attribute_opt(e, "shellinertia") {
    geom.shellinertia = Some(si == "true");
}
```

In geom defaults parsing (after `user`, in `parse_geom_defaults()`):

```rust
if let Some(si) = get_attribute_opt(e, "shellinertia") {
    defaults.shellinertia = Some(si == "true");
}
```

For `exactmeshinertia` (~line 484): keep parsing, add deprecation warning:

```rust
if let Some(emi) = get_attribute_opt(e, "exactmeshinertia") {
    compiler.exactmeshinertia = emi == "true";
    tracing::warn!(
        "exactmeshinertia is deprecated and has no effect; \
         use <mesh inertia=\"exact\"/> instead"
    );
}
```

### S3. Default class inheritance

**File:** `sim/L0/mjcf/src/defaults.rs`
**MuJoCo equivalent:** MJCF default class cascade
**Design decision:** Follow existing `.or()` merge pattern.

In `apply_to_mesh()` (~line 731), add `inertia` cascade:

```rust
pub fn apply_to_mesh(&self, mesh: &MjcfMesh) -> MjcfMesh {
    let mut result = mesh.clone();
    if let Some(defaults) = self.mesh_defaults(None) {
        if result.scale.is_none() {
            result.scale = defaults.scale;
        }
        if result.maxhullvert.is_none() {
            result.maxhullvert = defaults.maxhullvert;
        }
        if result.inertia.is_none() {
            result.inertia = defaults.inertia;
        }
    }
    result
}
```

In `merge_mesh_defaults()` (~line 993):

```rust
(Some(p), Some(c)) => Some(MjcfMeshDefaults {
    scale: c.scale.or(p.scale),
    maxhullvert: c.maxhullvert.or(p.maxhullvert),
    inertia: c.inertia.or(p.inertia),
}),
```

In `apply_to_geom()` (~line 233), add `shellinertia` cascade:

```rust
// After fluidcoef / user section (~line 326)
if result.shellinertia.is_none() {
    result.shellinertia = defaults.shellinertia;
}
```

In `merge_geom_defaults()` (~line 854):

```rust
(Some(p), Some(c)) => Some(MjcfGeomDefaults {
    // ... existing fields ...
    user: c.user.clone().or_else(|| p.user.clone()),
    shellinertia: c.shellinertia.or(p.shellinertia),
}),
```

### S4. Shell mesh inertia algorithm

**File:** `sim/L0/mjcf/src/builder/mesh.rs`
**MuJoCo equivalent:** `ComputeInertia()` in `user_mesh.cc`, shell path
**Design decision:** New function `compute_mesh_inertia_shell()` parallel
to `compute_mesh_inertia()`. Returns `(total_area, com, inertia_at_com)`
where the first element is surface area (not volume). The type alias
`MeshProps` is unchanged — callers interpret the first element as "measure"
(volume or area) and use it for density scaling.

```rust
/// Compute shell (surface-area-weighted) mass properties of a triangle mesh.
///
/// Returns `(total_area, com, inertia_at_com)` where:
/// - `total_area` is the total surface area
/// - `com` is the area-weighted centroid
/// - `inertia_at_com` is the inertia tensor about COM at unit surface density
///
/// MuJoCo ref: `ComputeInertia()` in `user_mesh.cc`, shell path.
/// Uses surface integral formulas:
///   ∫_T xᵢ² dA = (A/6)(aᵢ² + bᵢ² + cᵢ² + aᵢbᵢ + aᵢcᵢ + bᵢcᵢ)
///   ∫_T xᵢxⱼ dA = (A/12)(2aᵢaⱼ + 2bᵢbⱼ + 2cᵢcⱼ + mixed terms)
#[allow(clippy::suspicious_operation_groupings)]
pub fn compute_mesh_inertia_shell(mesh: &TriangleMeshData) -> MeshProps {
    let vertices = mesh.vertices();
    let triangles = mesh.triangles();

    let mut total_area = 0.0;
    let mut com_accum = Vector3::zeros();
    let mut xx = 0.0;
    let mut yy = 0.0;
    let mut zz = 0.0;
    let mut xy = 0.0;
    let mut xz = 0.0;
    let mut yz = 0.0;

    for tri in triangles {
        let a = vertices[tri.v0].coords;
        let b = vertices[tri.v1].coords;
        let c = vertices[tri.v2].coords;

        // Triangle area
        let cross = (b - a).cross(&(c - a));
        let area = cross.norm() * 0.5;
        total_area += area;

        // COM contribution: area-weighted centroid
        com_accum += area * (a + b + c) / 3.0;

        // Surface integral: ∫xᵢ² dA = (A/6)(aᵢ² + bᵢ² + cᵢ² + aᵢbᵢ + aᵢcᵢ + bᵢcᵢ)
        let f6 = area / 6.0;
        let f12 = area / 12.0;

        xx += f6 * (a.x*a.x + b.x*b.x + c.x*c.x + a.x*b.x + a.x*c.x + b.x*c.x);
        yy += f6 * (a.y*a.y + b.y*b.y + c.y*c.y + a.y*b.y + a.y*c.y + b.y*c.y);
        zz += f6 * (a.z*a.z + b.z*b.z + c.z*c.z + a.z*b.z + a.z*c.z + b.z*c.z);

        xy += f12 * (2.0*a.x*a.y + 2.0*b.x*b.y + 2.0*c.x*c.y
                     + a.x*b.y + a.y*b.x + a.x*c.y + a.y*c.x + b.x*c.y + b.y*c.x);
        xz += f12 * (2.0*a.x*a.z + 2.0*b.x*b.z + 2.0*c.x*c.z
                     + a.x*b.z + a.z*b.x + a.x*c.z + a.z*c.x + b.x*c.z + b.z*c.x);
        yz += f12 * (2.0*a.y*a.z + 2.0*b.y*b.z + 2.0*c.y*c.z
                     + a.y*b.z + a.z*b.y + a.y*c.z + a.z*c.y + b.y*c.z + b.z*c.y);
    }

    if total_area < 1e-10 {
        // Degenerate mesh — same fallback as compute_mesh_inertia()
        let (aabb_min, aabb_max) = mesh.aabb();
        let extents = aabb_max - aabb_min;
        let area = 2.0 * (extents.x*extents.y + extents.x*extents.z + extents.y*extents.z);
        let com = nalgebra::center(&aabb_min, &aabb_max).coords;
        let c = area / 12.0;
        let inertia = Matrix3::from_diagonal(&Vector3::new(
            c * (extents.y.powi(2) + extents.z.powi(2)),
            c * (extents.x.powi(2) + extents.z.powi(2)),
            c * (extents.x.powi(2) + extents.y.powi(2)),
        ));
        return (area, com, inertia);
    }

    let com = com_accum / total_area;

    let i_origin = Matrix3::new(
        yy + zz, -xy, -xz,
        -xy, xx + zz, -yz,
        -xz, -yz, xx + yy,
    );

    // Shift to COM via PAT: I_com = I_origin - SA * (d·d I₃ − d⊗d)
    let d = com;
    let d_sq = d.dot(&d);
    let parallel_shift = total_area * (Matrix3::identity() * d_sq - d * d.transpose());
    let i_com = i_origin - parallel_shift;

    (total_area, com, i_com)
}
```

### S5. Legacy mesh inertia algorithm

**File:** `sim/L0/mjcf/src/builder/mesh.rs`
**MuJoCo equivalent:** `ComputeInertia()` in `user_mesh.cc`, legacy path
**Design decision:** New function `compute_mesh_inertia_legacy()`. Identical
to `compute_mesh_inertia()` but uses `det.abs()` instead of `det` for
tetrahedron volumes and second-moment accumulation.

```rust
/// Compute legacy mass properties using absolute tetrahedron volumes.
///
/// Identical to `compute_mesh_inertia()` except all determinant-derived
/// values use `det.abs()` instead of signed `det`. This overcounts
/// non-convex regions where tetrahedra have negative signed volume.
///
/// MuJoCo ref: `ComputeInertia()` legacy path in `user_mesh.cc`.
#[allow(clippy::suspicious_operation_groupings)]
pub fn compute_mesh_inertia_legacy(mesh: &TriangleMeshData) -> MeshProps {
    let vertices = mesh.vertices();
    let triangles = mesh.triangles();

    let mut total_volume = 0.0;
    let mut com_accum = Vector3::zeros();
    let mut xx = 0.0;
    let mut yy = 0.0;
    let mut zz = 0.0;
    let mut xy = 0.0;
    let mut xz = 0.0;
    let mut yz = 0.0;

    for tri in triangles {
        let a = vertices[tri.v0].coords;
        let b = vertices[tri.v1].coords;
        let c = vertices[tri.v2].coords;

        let det = a.cross(&b).dot(&c);
        let det_abs = det.abs();  // KEY DIFFERENCE from exact mode
        let vol = det_abs / 6.0;
        total_volume += vol;

        com_accum += vol * (a + b + c) / 4.0;

        let f60 = det_abs / 60.0;
        let f120 = det_abs / 120.0;

        xx += f60 * (a.x*a.x + b.x*b.x + c.x*c.x + a.x*b.x + a.x*c.x + b.x*c.x);
        yy += f60 * (a.y*a.y + b.y*b.y + c.y*c.y + a.y*b.y + a.y*c.y + b.y*c.y);
        zz += f60 * (a.z*a.z + b.z*b.z + c.z*c.z + a.z*b.z + a.z*c.z + b.z*c.z);

        xy += f120 * (2.0*a.x*a.y + 2.0*b.x*b.y + 2.0*c.x*c.y
                      + a.x*b.y + a.y*b.x + a.x*c.y + a.y*c.x + b.x*c.y + b.y*c.x);
        xz += f120 * (2.0*a.x*a.z + 2.0*b.x*b.z + 2.0*c.x*c.z
                      + a.x*b.z + a.z*b.x + a.x*c.z + a.z*c.x + b.x*c.z + b.z*c.x);
        yz += f120 * (2.0*a.y*a.z + 2.0*b.y*b.z + 2.0*c.y*c.z
                      + a.y*b.z + a.z*b.y + a.y*c.z + a.z*c.y + b.y*c.z + b.z*c.y);
    }

    if total_volume < 1e-10 {
        let (aabb_min, aabb_max) = mesh.aabb();
        let extents = aabb_max - aabb_min;
        let volume = extents.x * extents.y * extents.z;
        let com = nalgebra::center(&aabb_min, &aabb_max).coords;
        let c = volume / 12.0;
        let inertia = Matrix3::from_diagonal(&Vector3::new(
            c * (extents.y.powi(2) + extents.z.powi(2)),
            c * (extents.x.powi(2) + extents.z.powi(2)),
            c * (extents.x.powi(2) + extents.y.powi(2)),
        ));
        return (volume, com, inertia);
    }

    let com = com_accum / total_volume;

    let i_origin = Matrix3::new(
        yy + zz, -xy, -xz,
        -xy, xx + zz, -yz,
        -xz, -yz, xx + yy,
    );

    let d = com;
    let d_sq = d.dot(&d);
    let parallel_shift = total_volume * (Matrix3::identity() * d_sq - d * d.transpose());
    let i_com = i_origin - parallel_shift;

    (total_volume, com, i_com)
}
```

### S6. Convex mesh inertia dispatch

**File:** `sim/L0/mjcf/src/builder/mesh.rs`
**MuJoCo equivalent:** `ComputeInertia()` convex path in `user_mesh.cc`
**Design decision:** New function `compute_mesh_inertia_on_hull()` that
runs the exact algorithm on the convex hull's vertex/face data. Uses a
helper `compute_inertia_from_verts_faces()` to avoid constructing a
temporary `TriangleMeshData` (and its BVH).

```rust
/// Compute exact (volumetric) inertia on the convex hull of a mesh.
///
/// Falls back to `compute_mesh_inertia()` on the original mesh if the
/// hull is not available.
pub fn compute_mesh_inertia_on_hull(mesh: &TriangleMeshData) -> MeshProps {
    match mesh.convex_hull() {
        Some(hull) => compute_inertia_from_verts_faces(&hull.vertices, &hull.faces),
        None => compute_mesh_inertia(mesh),
    }
}

/// Compute exact (volumetric) inertia from raw vertex/face arrays.
///
/// Same signed-tetrahedron algorithm as `compute_mesh_inertia()` but
/// operates on raw arrays instead of `TriangleMeshData`.
#[allow(clippy::suspicious_operation_groupings)]
fn compute_inertia_from_verts_faces(
    vertices: &[Point3<f64>],
    faces: &[[usize; 3]],
) -> MeshProps {
    let mut total_volume = 0.0;
    let mut com_accum = Vector3::zeros();
    let mut xx = 0.0;
    let mut yy = 0.0;
    let mut zz = 0.0;
    let mut xy = 0.0;
    let mut xz = 0.0;
    let mut yz = 0.0;

    for face in faces {
        let a = vertices[face[0]].coords;
        let b = vertices[face[1]].coords;
        let c = vertices[face[2]].coords;

        let det = a.cross(&b).dot(&c);
        let vol = det / 6.0;
        total_volume += vol;
        com_accum += vol * (a + b + c) / 4.0;

        let f60 = det / 60.0;
        let f120 = det / 120.0;

        xx += f60 * (a.x*a.x + b.x*b.x + c.x*c.x + a.x*b.x + a.x*c.x + b.x*c.x);
        yy += f60 * (a.y*a.y + b.y*b.y + c.y*c.y + a.y*b.y + a.y*c.y + b.y*c.y);
        zz += f60 * (a.z*a.z + b.z*b.z + c.z*c.z + a.z*b.z + a.z*c.z + b.z*c.z);

        xy += f120 * (2.0*a.x*a.y + 2.0*b.x*b.y + 2.0*c.x*c.y
                      + a.x*b.y + a.y*b.x + a.x*c.y + a.y*c.x + b.x*c.y + b.y*c.x);
        xz += f120 * (2.0*a.x*a.z + 2.0*b.x*b.z + 2.0*c.x*c.z
                      + a.x*b.z + a.z*b.x + a.x*c.z + a.z*c.x + b.x*c.z + b.z*c.x);
        yz += f120 * (2.0*a.y*a.z + 2.0*b.y*b.z + 2.0*c.y*c.z
                      + a.y*b.z + a.z*b.y + a.y*c.z + a.z*c.y + b.y*c.z + b.z*c.y);
    }

    if total_volume.abs() < 1e-10 {
        return (0.0, Vector3::zeros(), Matrix3::zeros());
    }

    let com = com_accum / total_volume;

    let i_origin = Matrix3::new(
        yy + zz, -xy, -xz,
        -xy, xx + zz, -yz,
        -xz, -yz, xx + yy,
    );

    let d = com;
    let d_sq = d.dot(&d);
    let parallel_shift = total_volume * (Matrix3::identity() * d_sq - d * d.transpose());
    let i_com = i_origin - parallel_shift;

    (total_volume, com, i_com)
}
```

### S7. Primitive shell inertia formulas

**File:** `sim/L0/mjcf/src/builder/geom.rs`
**MuJoCo equivalent:** `SetGeom()` in `user_mesh.cc`, shell path
**Design decision:** Add `compute_geom_shell_mass()` and
`compute_geom_shell_inertia()` parallel to the existing solid functions.
These are called when `shellinertia == Some(true)` and the geom is NOT a
mesh type.

```rust
/// Compute shell (surface-area-based) mass of a primitive geom.
///
/// Mass = density × surface_area.
pub fn compute_geom_shell_mass(geom: &MjcfGeom) -> f64 {
    if let Some(mass) = geom.mass {
        return mass;
    }

    let density = geom.density.unwrap_or(1000.0);
    let sa = match geom.geom_type.unwrap_or(MjcfGeomType::Sphere) {
        MjcfGeomType::Sphere => {
            let r = geom.size.first().copied().unwrap_or(0.1);
            4.0 * std::f64::consts::PI * r.powi(2)
        }
        MjcfGeomType::Box => {
            let a = geom.size.first().copied().unwrap_or(0.1);
            let b = geom.size.get(1).copied().unwrap_or(a);
            let c = geom.size.get(2).copied().unwrap_or(b);
            8.0 * (a * b + a * c + b * c)
        }
        MjcfGeomType::Cylinder => {
            let r = geom.size.first().copied().unwrap_or(0.1);
            let h = geom.size.get(1).copied().unwrap_or(0.1) * 2.0;
            2.0 * std::f64::consts::PI * r * h + 2.0 * std::f64::consts::PI * r.powi(2)
        }
        MjcfGeomType::Capsule => {
            let r = geom.size.first().copied().unwrap_or(0.1);
            let h = geom.size.get(1).copied().unwrap_or(0.1) * 2.0;
            2.0 * std::f64::consts::PI * r * h + 4.0 * std::f64::consts::PI * r.powi(2)
        }
        MjcfGeomType::Ellipsoid => {
            let a = geom.size.first().copied().unwrap_or(0.1);
            let b = geom.size.get(1).copied().unwrap_or(a);
            let c = geom.size.get(2).copied().unwrap_or(b);
            ellipsoid_surface_area(a, b, c)
        }
        _ => 0.001,
    };

    density * sa
}

/// Knud Thomsen approximation for ellipsoid surface area.
/// SA ≈ 4π × ((a^p b^p + a^p c^p + b^p c^p) / 3)^(1/p), p ≈ 1.6075.
fn ellipsoid_surface_area(a: f64, b: f64, c: f64) -> f64 {
    let p = 1.6075;
    let ap = a.powf(p);
    let bp = b.powf(p);
    let cp = c.powf(p);
    let inner = (ap * bp + ap * cp + bp * cp) / 3.0;
    4.0 * std::f64::consts::PI * inner.powf(1.0 / p)
}

/// Compute shell inertia tensor of a primitive geom.
pub fn compute_geom_shell_inertia(geom: &MjcfGeom) -> Matrix3<f64> {
    let mass = compute_geom_shell_mass(geom);

    match geom.geom_type.unwrap_or(MjcfGeomType::Sphere) {
        MjcfGeomType::Sphere => {
            let r = geom.size.first().copied().unwrap_or(0.1);
            let i = (2.0 / 3.0) * mass * r.powi(2);
            Matrix3::from_diagonal(&Vector3::new(i, i, i))
        }
        MjcfGeomType::Box => {
            let a = geom.size.first().copied().unwrap_or(0.1);
            let b = geom.size.get(1).copied().unwrap_or(a);
            let c = geom.size.get(2).copied().unwrap_or(b);
            let density = if let Some(m) = geom.mass {
                m / (8.0 * (a*b + a*c + b*c))
            } else {
                geom.density.unwrap_or(1000.0)
            };
            shell_box_inertia(a, b, c, density)
        }
        MjcfGeomType::Cylinder => {
            let r = geom.size.first().copied().unwrap_or(0.1);
            let h = geom.size.get(1).copied().unwrap_or(0.1) * 2.0;
            let density = if let Some(m) = geom.mass {
                let sa = 2.0 * std::f64::consts::PI * r * h
                       + 2.0 * std::f64::consts::PI * r.powi(2);
                m / sa
            } else {
                geom.density.unwrap_or(1000.0)
            };
            shell_cylinder_inertia(r, h, density)
        }
        MjcfGeomType::Capsule => {
            let r = geom.size.first().copied().unwrap_or(0.1);
            let h = geom.size.get(1).copied().unwrap_or(0.1) * 2.0;
            let density = if let Some(m) = geom.mass {
                let sa = 2.0 * std::f64::consts::PI * r * h
                       + 4.0 * std::f64::consts::PI * r.powi(2);
                m / sa
            } else {
                geom.density.unwrap_or(1000.0)
            };
            shell_capsule_inertia(r, h, density)
        }
        MjcfGeomType::Ellipsoid => {
            let a = geom.size.first().copied().unwrap_or(0.1);
            let b = geom.size.get(1).copied().unwrap_or(a);
            let c = geom.size.get(2).copied().unwrap_or(b);
            shell_ellipsoid_inertia(a, b, c, mass)
        }
        _ => Matrix3::from_diagonal(&Vector3::new(0.001, 0.001, 0.001)),
    }
}
```

**Shell formula implementations:**

```rust
/// Box shell inertia via face decomposition + PAT.
fn shell_box_inertia(a: f64, b: f64, c: f64, density: f64) -> Matrix3<f64> {
    // ±x faces (area 4bc each)
    let m_x = density * 4.0 * b * c;
    let ix_x = m_x * (b*b + c*c) / 3.0;
    let iy_x = m_x * (a*a + c*c / 3.0);
    let iz_x = m_x * (a*a + b*b / 3.0);

    // ±y faces (area 4ac each)
    let m_y = density * 4.0 * a * c;
    let ix_y = m_y * (b*b + c*c / 3.0);
    let iy_y = m_y * (a*a + c*c) / 3.0;
    let iz_y = m_y * (b*b + a*a / 3.0);

    // ±z faces (area 4ab each)
    let m_z = density * 4.0 * a * b;
    let ix_z = m_z * (b*b / 3.0 + c*c);
    let iy_z = m_z * (a*a / 3.0 + c*c);
    let iz_z = m_z * (a*a + b*b) / 3.0;

    Matrix3::from_diagonal(&Vector3::new(
        2.0 * (ix_x + ix_y + ix_z),
        2.0 * (iy_x + iy_y + iy_z),
        2.0 * (iz_x + iz_y + iz_z),
    ))
}

/// Cylinder shell inertia: lateral shell + 2 disk caps.
fn shell_cylinder_inertia(r: f64, h: f64, density: f64) -> Matrix3<f64> {
    let pi = std::f64::consts::PI;

    // Lateral shell
    let m_lat = density * 2.0 * pi * r * h;
    let ix_lat = m_lat * (r*r / 2.0 + h*h / 12.0);
    let iz_lat = m_lat * r*r;

    // End cap (thin disk), each
    let m_cap = density * pi * r*r;
    let ix_cap_own = m_cap * r*r / 4.0;
    let iz_cap_own = m_cap * r*r / 2.0;
    let ix_cap = ix_cap_own + m_cap * (h / 2.0).powi(2);

    Matrix3::from_diagonal(&Vector3::new(
        ix_lat + 2.0 * ix_cap,
        ix_lat + 2.0 * ix_cap,
        iz_lat + 2.0 * iz_cap_own,
    ))
}

/// Capsule shell inertia: open cylinder lateral + 2 hemisphere shells.
fn shell_capsule_inertia(r: f64, h: f64, density: f64) -> Matrix3<f64> {
    let pi = std::f64::consts::PI;

    // Lateral (open cylinder — no end caps)
    let m_lat = density * 2.0 * pi * r * h;
    let ix_lat = m_lat * (r*r / 2.0 + h*h / 12.0);
    let iz_lat = m_lat * r*r;

    // Hemisphere shell (each)
    let m_hemi = density * 2.0 * pi * r*r;
    // Inertia about own COM (COM is at r/2 from equator)
    let ix_hemi_own = m_hemi * 5.0 * r*r / 12.0;
    let iz_hemi = 2.0 / 3.0 * m_hemi * r*r;
    // PAT: distance from hemisphere COM to capsule center
    let d = h / 2.0 + r / 2.0;
    let ix_hemi = ix_hemi_own + m_hemi * d*d;

    Matrix3::from_diagonal(&Vector3::new(
        ix_lat + 2.0 * ix_hemi,
        ix_lat + 2.0 * ix_hemi,
        iz_lat + 2.0 * iz_hemi,
    ))
}

/// Ellipsoid shell inertia via numerical quadrature on parametric surface.
///
/// Computes ∫∫_S xᵢ² dS via Gauss-Legendre quadrature on (θ, φ) ∈ [0,π]×[0,2π].
fn shell_ellipsoid_inertia(a: f64, b: f64, c: f64, mass: f64) -> Matrix3<f64> {
    let n_theta = 64_usize;
    let n_phi = 128_usize;

    // Gauss-Legendre nodes/weights for [0, π] (θ) and [0, 2π] (φ)
    let (theta_nodes, theta_weights) = gauss_legendre_01(n_theta, 0.0, std::f64::consts::PI);
    let (phi_nodes, phi_weights) = gauss_legendre_01(n_phi, 0.0, 2.0 * std::f64::consts::PI);

    let mut sa_num = 0.0;
    let mut int_xx = 0.0;
    let mut int_yy = 0.0;
    let mut int_zz = 0.0;

    for i in 0..n_theta {
        let theta = theta_nodes[i];
        let wt = theta_weights[i];
        let sin_t = theta.sin();
        let cos_t = theta.cos();

        for j in 0..n_phi {
            let phi = phi_nodes[j];
            let wp = phi_weights[j];
            let sin_p = phi.sin();
            let cos_p = phi.cos();

            // Surface element magnitude
            let e = (c*c * sin_t*sin_t * (b*b * cos_p*cos_p + a*a * sin_p*sin_p)
                     + a*a * b*b * cos_t*cos_t).sqrt();
            let ds = sin_t * e * wt * wp;

            sa_num += ds;
            int_xx += (a * sin_t * cos_p).powi(2) * ds;
            int_yy += (b * sin_t * sin_p).powi(2) * ds;
            int_zz += (c * cos_t).powi(2) * ds;
        }
    }

    // Normalize: I_x = m × (⟨y²⟩ + ⟨z²⟩) where ⟨x²⟩ = ∫x² dS / SA
    let ix = mass * (int_yy + int_zz) / sa_num;
    let iy = mass * (int_xx + int_zz) / sa_num;
    let iz = mass * (int_xx + int_yy) / sa_num;

    Matrix3::from_diagonal(&Vector3::new(ix, iy, iz))
}

/// Gauss-Legendre quadrature nodes and weights mapped to [lo, hi].
fn gauss_legendre_01(n: usize, lo: f64, hi: f64) -> (Vec<f64>, Vec<f64>) {
    // Compute Gauss-Legendre nodes/weights on [-1, 1], then map to [lo, hi].
    // Standard Golub-Welsch algorithm or tabulated values.
    // For n=64/128, use iterative eigenvalue method.
    let (std_nodes, std_weights) = gauss_legendre_standard(n);
    let mid = (lo + hi) / 2.0;
    let half = (hi - lo) / 2.0;
    let nodes: Vec<f64> = std_nodes.iter().map(|&x| mid + half * x).collect();
    let weights: Vec<f64> = std_weights.iter().map(|&w| w * half).collect();
    (nodes, weights)
}
```

**Note:** The `gauss_legendre_standard()` function computes GL nodes/weights
on [-1,1] via the Golub-Welsch eigenvalue method. This is a standard
numerical method (~30 lines of Rust using nalgebra's eigendecomposition).
The implementation should be placed in a helper module or inline in
`builder/geom.rs`.

### S8. Dispatch integration

**File:** `sim/L0/mjcf/src/builder/geom.rs`, `sim/L0/mjcf/src/builder/mass.rs`
**MuJoCo equivalent:** `compute_geom_mass()` / inertia mode dispatch
**Design decision:** Thread mesh inertia mode through `MeshProps`
computation. Add `mesh_inertia_modes` parameter to
`compute_inertia_from_geoms()`. Fix missing Ellipsoid volume in
`compute_geom_mass()`.

**8a. Fix missing Ellipsoid in `compute_geom_mass()`** (geom.rs ~line 311):

Add before the `_ =>` fallback:

```rust
MjcfGeomType::Ellipsoid => {
    let a = geom.size.first().copied().unwrap_or(0.1);
    let b = geom.size.get(1).copied().unwrap_or(a);
    let c = geom.size.get(2).copied().unwrap_or(b);
    (4.0 / 3.0) * std::f64::consts::PI * a * b * c
}
```

**8b. Add mode-aware mass dispatch in `compute_geom_mass()`**:

When `shellinertia == Some(true)` and geom is NOT mesh, delegate to
`compute_geom_shell_mass()`:

```rust
pub fn compute_geom_mass(geom: &MjcfGeom, mesh_props: Option<&MeshProps>) -> f64 {
    if let Some(mass) = geom.mass {
        return mass;
    }

    // Shell inertia for primitive geoms
    if geom.shellinertia == Some(true) {
        return compute_geom_shell_mass(geom);
    }

    // Volume-based (existing logic + Ellipsoid fix)
    let volume = match geom.geom_type.unwrap_or(MjcfGeomType::Sphere) {
        // ... existing arms ...
        MjcfGeomType::Ellipsoid => {
            let a = geom.size.first().copied().unwrap_or(0.1);
            let b = geom.size.get(1).copied().unwrap_or(a);
            let c = geom.size.get(2).copied().unwrap_or(b);
            (4.0 / 3.0) * std::f64::consts::PI * a * b * c
        }
        _ => 0.001,
    };
    geom.density.unwrap_or(1000.0) * volume
}
```

**8c. Add mode-aware inertia dispatch in `compute_geom_inertia()`**:

```rust
pub fn compute_geom_inertia(geom: &MjcfGeom, mesh_props: Option<&MeshProps>) -> Matrix3<f64> {
    // Shell inertia for primitive geoms
    if geom.shellinertia == Some(true) {
        return compute_geom_shell_inertia(geom);
    }

    // ... existing solid inertia logic (unchanged) ...
}
```

**8d. Mode-aware `MeshProps` computation in `compute_inertia_from_geoms()`**:

Add a new helper function that dispatches mesh inertia by mode:

```rust
/// Compute mesh inertia properties using the specified mode.
pub fn compute_mesh_inertia_by_mode(
    mesh: &TriangleMeshData,
    mode: MeshInertia,
) -> MeshProps {
    match mode {
        MeshInertia::Convex => compute_mesh_inertia_on_hull(mesh),
        MeshInertia::Exact => compute_mesh_inertia(mesh),
        MeshInertia::Legacy => compute_mesh_inertia_legacy(mesh),
        MeshInertia::Shell => compute_mesh_inertia_shell(mesh),
    }
}
```

Update `compute_inertia_from_geoms()` in `mass.rs` to accept
`mesh_inertia_modes: &[MeshInertia]` (indexed by mesh_id) and pass the
mode to the dispatch function:

```rust
pub fn compute_inertia_from_geoms(
    geoms: &[MjcfGeom],
    mesh_lookup: &HashMap<String, usize>,
    mesh_data: &[Arc<TriangleMeshData>],
    mesh_inertia_modes: &[MeshInertia],  // NEW parameter
) -> (f64, Vector3<f64>, Vector3<f64>, UnitQuaternion<f64>) {
    // ...
    let mesh_props: Vec<Option<MeshProps>> = geoms
        .iter()
        .map(|geom| {
            resolve_mesh(geom, mesh_lookup, mesh_data).map(|mesh| {
                let mesh_id = geom.mesh.as_ref()
                    .and_then(|name| mesh_lookup.get(name))
                    .copied()
                    .unwrap_or(0);
                let mode = mesh_inertia_modes.get(mesh_id)
                    .copied()
                    .unwrap_or(MeshInertia::Convex);
                compute_mesh_inertia_by_mode(&mesh, mode)
            })
        })
        .collect();
    // ... rest unchanged ...
}
```

The caller (in `builder/body.rs` or `builder/mod.rs`) must pass
`&self.mesh_inertia_modes` — a new `Vec<MeshInertia>` on the builder,
populated during `process_mesh()`:

```rust
// In process_mesh(), after storing mesh_data:
let mode = mjcf_mesh.inertia.unwrap_or(MeshInertia::Convex);
self.mesh_inertia_modes.push(mode);
```

**8e. Negative volume validation for exact mode:**

When `compute_mesh_inertia()` (exact mode) produces a negative volume, the
mesh has misoriented triangles. MuJoCo rejects this at compile time (EGT-13).
Add validation after `compute_mesh_inertia_by_mode()` returns for
`MeshInertia::Exact`:

```rust
// In compute_inertia_from_geoms() or process_mesh(), after mesh inertia computation:
if mode == MeshInertia::Exact {
    let (volume, _, _) = &props;
    if *volume < 0.0 {
        return Err(ModelConversionError {
            message: format!(
                "mesh volume is negative (misoriented triangles): {}",
                mesh_name
            ),
        });
    }
}
```

**Note:** The existing `compute_mesh_inertia()` returns signed volume. This
validation is only needed for `Exact` mode — `Legacy` uses `det.abs()` (always
positive), `Convex` runs on the convex hull (always positive for valid hulls),
and `Shell` computes area (always positive). If the existing code already
performs this check, this step is a no-op and AC17 becomes a regression test.

### S9. Validation: shellinertia rejection on mesh geoms

**File:** `sim/L0/mjcf/src/builder/geom.rs` (or validation pass)
**MuJoCo equivalent:** MuJoCo 3.5.0 compile-time error
**Design decision:** Check `shellinertia` on mesh geoms during model
building (in `process_geom()` or during mass computation) and return error.

```rust
// In process_geom() or compute_geom_mass(), early in mesh handling:
if geom.geom_type == Some(MjcfGeomType::Mesh) && geom.shellinertia == Some(true) {
    return Err(ModelConversionError {
        message: format!(
            "geom '{}': for mesh geoms, inertia should be specified in the mesh asset",
            geom.name.as_deref().unwrap_or("<unnamed>")
        ),
    });
}
```

---

## Acceptance Criteria

### AC1: Default mesh inertia mode is Convex *(runtime test — MuJoCo-verified)*
**Given:** Unit cube mesh, density=1000, no `inertia` attribute
**After:** Model load
**Assert:** `body_mass[1]` = 1000.0 ± 1e-6, `body_inertia[1]` = [166.667, 166.667, 166.667] ± 0.01
**Field:** Model body mass and inertia arrays
**Note:** Convex == Exact for convex meshes. Matches EGT-1.

### AC2: Shell mesh inertia — unit cube *(runtime test — MuJoCo-verified)*
**Given:** Unit cube mesh with `inertia="shell"`, density=1000
**After:** Model load
**Assert:** `body_mass[1]` = 6000.0 ± 1e-6, `body_inertia[1]` = [1666.667, 1666.667, 1666.667] ± 0.01
**Field:** Model body mass and inertia

### AC3: Shell mesh inertia — explicit mass override *(runtime test — MuJoCo-verified)*
**Given:** Unit cube mesh with `inertia="shell"`, geom `mass="5.0"`
**After:** Model load
**Assert:** `body_mass[1]` = 5.0 ± 1e-6, `body_inertia[1]` = [1.389, 1.389, 1.389] ± 0.01
**Field:** Model body mass and inertia

### AC4: Sphere shell inertia *(runtime test — MuJoCo-verified)*
**Given:** Sphere geom, r=1, density=1000, `shellinertia="true"`
**After:** Model load
**Assert:** `body_mass[1]` = 12566.37 ± 0.1, `body_inertia[1]` = [8377.58, 8377.58, 8377.58] ± 0.1
**Field:** Model body mass and inertia

### AC5: Box shell inertia *(runtime test — MuJoCo-verified)*
**Given:** Box geom, half-extents (1,2,3), density=1000, `shellinertia="true"`
**After:** Model load
**Assert:** `body_mass[1]` = 88000.0 ± 1e-6, `body_inertia[1]` = [541333.33, 421333.33, 242666.67] ± 1.0
**Field:** Model body mass and inertia

### AC6: Cylinder shell inertia *(runtime test — MuJoCo-verified)*
**Given:** Cylinder geom, r=1, half-h=2, density=1000, `shellinertia="true"`
**After:** Model load
**Assert:** `body_mass[1]` = 31415.93 ± 0.1, `body_inertia[1]` = [72780.23, 72780.23, 28274.33] ± 0.1
**Field:** Model body mass and inertia

### AC7: Capsule shell inertia *(runtime test — MuJoCo-verified)*
**Given:** Capsule geom, r=1, half-h=2, density=1000, `shellinertia="true"`
**After:** Model load
**Assert:** `body_mass[1]` = 37699.11 ± 0.1, `body_inertia[1]` = [129852.50, 129852.50, 33510.32] ± 1.0
**Field:** Model body mass and inertia

### AC8: Ellipsoid shell inertia *(runtime test — MuJoCo-verified)*
**Given:** Ellipsoid geom, semi-axes (1,2,3), density=1000, `shellinertia="true"`
**After:** Model load
**Assert:** `body_mass[1]` = 48971.93 ± 500 (1% tolerance), `body_inertia[1]` = [180751.03, 140683.07, 81026.34] ± 1% relative
**Field:** Model body mass and inertia
**Note:** Wider tolerance per AD-1 (Thomsen SA approximation). MuJoCo values from EGT-8.

### AC9: Convex mode on non-convex mesh *(runtime test — MuJoCo-verified)*
**Given:** L-shape mesh, `inertia="convex"`, density=1000
**After:** Model load
**Assert:** `body_mass[1]` = 3500.0 ± 1.0
**Field:** Model body mass
**Note:** Convex hull volume > original mesh volume for non-convex shapes. EGT-12.

### AC10: Legacy mode *(runtime test — MuJoCo-verified)*
**Given:** L-shape mesh, `inertia="legacy"`, density=1000
**After:** Model load
**Assert:** `body_mass[1]` = 3000.0 ± 1.0, `body_inertia[1]` = [1833.33, 1500.0, 833.33] ± 1.0
**Field:** Model body mass and inertia

### AC11: shellinertia rejected on mesh geom *(runtime test)*
**Given:** Mesh geom with `shellinertia="true"`
**After:** Model load attempt
**Assert:** Load fails with error containing "for mesh geoms, inertia should be specified in the mesh asset"
**Field:** Error message

### AC12: Default class inheritance for mesh inertia *(runtime test — MuJoCo-verified)*
**Given:** `<default><mesh inertia="shell"/></default>`, cube mesh with no explicit `inertia`
**After:** Model load
**Assert:** `body_mass[1]` = 6000.0 ± 1e-6 (shell inertia inherited)
**Field:** Model body mass

### AC13: Ellipsoid solid volume fix *(runtime test — analytically derived)*
**Given:** Ellipsoid geom, semi-axes (1,2,3), density=1000, NO `shellinertia`
**After:** Model load
**Assert:** `body_mass[1]` = 1000 × 4/3 × π × 1 × 2 × 3 = 25132.74 ± 0.1
**Field:** Model body mass

### AC14: exactmeshinertia warning *(runtime test)*
**Given:** `<compiler exactmeshinertia="true"/>`
**After:** Model load
**Assert:** Model loads successfully (no error). Warning emitted (verified via test output or `tracing` subscriber).
**Field:** Parse success

### AC15: Existing exact inertia tests unchanged *(regression — code review)*
**Assert:** All 10 tests in `sim/L0/tests/integration/exactmeshinertia.rs`
(AC1–AC10) pass without value changes. The default mode is now `Convex`,
but the test meshes are convex (cubes), so `Convex == Exact` numerically.

### AC16: inertia attribute parsed on mesh *(runtime test)*
**Given:** `<mesh name="m" inertia="shell" vertex="..." face="..."/>`
**After:** Model load
**Assert:** Shell inertia values produced (same as AC2)
**Field:** Model body mass

### AC17: Exact mode rejects misoriented mesh *(runtime test — MuJoCo-verified)*
**Given:** Mesh with fully reversed face winding, `inertia="exact"`, density=1000
**After:** Model load attempt
**Assert:** Load fails with error containing "mesh volume is negative (misoriented triangles)"
**Field:** Error message
**Note:** Only triggers with fully reversed winding. Mixed normals produce "inconsistent orientation" — not tested here. Matches EGT-13.

### AC18: Default class inheritance for shellinertia *(runtime test)*
**Given:** `<default><geom shellinertia="true"/></default>`, sphere geom with no explicit `shellinertia`
**After:** Model load
**Assert:** `body_mass[1]` = 12566.37 ± 0.1 (shell mass, not solid 4188.79)
**Field:** Model body mass
**Note:** Validates that `shellinertia` cascades through `apply_to_geom()` / `merge_geom_defaults()`.

---

## AC → Test Traceability Matrix

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 (default convex) | T1 | Direct |
| AC2 (shell cube) | T2 | Direct |
| AC3 (shell mass override) | T3 | Direct |
| AC4 (sphere shell) | T4 | Direct |
| AC5 (box shell) | T5 | Direct |
| AC6 (cylinder shell) | T6 | Direct |
| AC7 (capsule shell) | T7 | Direct |
| AC8 (ellipsoid shell) | T8 | Direct |
| AC9 (convex non-convex) | T9 | Direct |
| AC10 (legacy) | T10 | Direct |
| AC11 (shellinertia mesh reject) | T11 | Direct |
| AC12 (default inheritance) | T12 | Direct |
| AC13 (ellipsoid volume fix) | T13 | Direct |
| AC14 (exactmeshinertia warn) | T14 | Direct |
| AC15 (exact regression) | T15 | Regression |
| AC16 (inertia parse) | T2 | Direct (covered by T2) |
| AC17 (exact misoriented error) | T18 | Direct |
| AC18 (shellinertia default inheritance) | T19 | Direct |

---

## Test Plan

All tests in `sim/L0/tests/integration/exactmeshinertia.rs` (extending
the existing test file, or a new `mesh_inertia_modes.rs` file).

### T1: Default mode is Convex → AC1

```rust
// Unit cube, density=1000, no inertia attribute.
// MuJoCo 3.5.0: mass=1000, I=[166.667, 166.667, 166.667].
// For convex meshes, Convex == Exact.
let mjcf = r#"<mujoco>
  <asset><mesh name="cube" vertex="..." face="..."/></asset>
  <worldbody><body>
    <geom type="mesh" mesh="cube" density="1000"/>
  </body></worldbody>
</mujoco>"#;
let model = load_model(mjcf).unwrap();
assert_relative_eq!(model.body_mass[1], 1000.0, epsilon = 1e-6);
```

### T2: Shell mesh inertia — unit cube → AC2, AC16

```rust
// Unit cube with inertia="shell", density=1000.
// MuJoCo 3.5.0: mass=6000, I=[1666.667, 1666.667, 1666.667].
let mjcf = r#"<mujoco>
  <asset><mesh name="cube" inertia="shell" vertex="..." face="..."/></asset>
  ...
</mujoco>"#;
// Assert mass=6000.0, I=[1666.667, 1666.667, 1666.667]
```

### T3: Shell mesh inertia — explicit mass → AC3

```rust
// Unit cube, inertia="shell", geom mass="5.0".
// MuJoCo 3.5.0: mass=5.0, I=[1.389, 1.389, 1.389].
// Scale: 5.0/6000 applied to shell inertia.
```

### T4: Sphere shell → AC4

```rust
// Sphere r=1, density=1000, shellinertia="true".
// MuJoCo 3.5.0: mass=12566.37, I=8377.58.
// Formula: m=ρ×4πr², I=2/3×m×r².
```

### T5: Box shell → AC5

```rust
// Box half-extents (1,2,3), density=1000, shellinertia="true".
// MuJoCo 3.5.0: mass=88000, I=[541333.33, 421333.33, 242666.67].
// Face decomposition algorithm.
```

### T6: Cylinder shell → AC6

```rust
// Cylinder r=1, half-h=2, density=1000, shellinertia="true".
// MuJoCo 3.5.0: mass=31415.93, I=[72780.23, 72780.23, 28274.33].
// Lateral + caps decomposition.
```

### T7: Capsule shell → AC7

```rust
// Capsule r=1, half-h=2, density=1000, shellinertia="true".
// MuJoCo 3.5.0: mass=37699.11, I=[129852.50, 129852.50, 33510.32].
// Lateral + hemisphere decomposition with correct PAT (d = h/2 + r/2).
```

### T8: Ellipsoid shell → AC8

```rust
// Ellipsoid semi-axes (1,2,3), density=1000, shellinertia="true".
// MuJoCo 3.5.0: mass=48971.93, I=[180751.03, 140683.07, 81026.34].
// Thomsen SA + quadrature. Tolerance 1% relative.
```

### T9: Convex mode on non-convex mesh → AC9

```rust
// L-shape mesh, inertia="convex", density=1000.
// MuJoCo 3.5.0: mass=3500.
// Convex hull of L-shape is a bounding box containing extra volume.
```

### T10: Legacy mode → AC10

```rust
// L-shape mesh, inertia="legacy", density=1000.
// MuJoCo 3.5.0: mass=3000, I=[1833.33, 1500.0, 833.33].
// Absolute tetrahedron volumes.
```

### T11: shellinertia rejected on mesh geom → AC11

```rust
// Mesh geom with shellinertia="true" → error.
let mjcf = r#"<mujoco>
  <asset><mesh name="cube" vertex="..." face="..."/></asset>
  <worldbody><body>
    <geom type="mesh" mesh="cube" shellinertia="true"/>
  </body></worldbody>
</mujoco>"#;
let result = load_model(mjcf);
assert!(result.is_err());
assert!(result.unwrap_err().to_string().contains("for mesh geoms"));
```

### T12: Default class inheritance → AC12

```rust
// <default><mesh inertia="shell"/></default>, cube mesh.
// Should produce shell inertia (mass=6000 for unit cube).
let mjcf = r#"<mujoco>
  <default><mesh inertia="shell"/></default>
  <asset><mesh name="cube" vertex="..." face="..."/></asset>
  ...
</mujoco>"#;
assert_relative_eq!(model.body_mass[1], 6000.0, epsilon = 1e-6);
```

### T13: Ellipsoid solid volume fix → AC13

```rust
// Ellipsoid semi-axes (1,2,3), density=1000, NO shellinertia.
// Expected mass = 1000 × 4/3 × π × 6 = 25132.74.
// This was previously broken (returned 1000 × 0.001 = 1.0).
```

### T14: exactmeshinertia deprecation → AC14

```rust
// <compiler exactmeshinertia="true"/> should parse without error.
// The attribute has no behavioral effect.
let mjcf = r#"<mujoco>
  <compiler exactmeshinertia="true"/>
  <asset><mesh name="cube" vertex="..." face="..."/></asset>
  ...
</mujoco>"#;
assert!(load_model(mjcf).is_ok());
```

### T15: Existing exact tests regression → AC15

```rust
// Run all 10 tests in exactmeshinertia.rs.
// All must pass unchanged. Default mode is now Convex, but test
// meshes are convex (cubes), so Convex == Exact numerically.
```

### Edge Case Inventory

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| Zero-area degenerate mesh + shell mode | Division by zero in COM/inertia | T16 | — |
| Misoriented mesh with exact mode | Must reject with negative volume error | T18 | AC17 |
| Convex mode on mesh that's already convex (cube) | Must equal exact mode | T1 | AC1, AC15 |
| Explicit mass override with shell density | Scale factor = mass/(density×area) | T3 | AC3 |
| `shellinertia` on mesh geom | Must error (MuJoCo 3.5.0 rejects) | T11 | AC11 |
| `shellinertia` on each of 5 primitive types | All must compute shell inertia | T4-T8 | AC4-AC8 |
| Default class inheritance for `<mesh inertia>` | Attribute must cascade | T12 | AC12 |
| Default class inheritance for `<geom shellinertia>` | Attribute must cascade through `apply_to_geom()` | T19 | AC18 |
| Multi-geom body mixing shell and solid geoms | Mass aggregation must be correct | T17 | — |

### T18: Exact mode rejects misoriented mesh → AC17

```rust
// Mesh with reversed face winding, inertia="exact".
// MuJoCo 3.5.0: "mesh volume is negative (misoriented triangles): {name}".
let mjcf = r#"<mujoco>
  <asset><mesh name="rev" inertia="exact" vertex="..." face="(reversed winding)"/></asset>
  <worldbody><body>
    <geom type="mesh" mesh="rev" density="1000"/>
  </body></worldbody>
</mujoco>"#;
let result = load_model(mjcf);
assert!(result.is_err());
assert!(result.unwrap_err().to_string().contains("mesh volume is negative"));
```

### T19: Default class inheritance for shellinertia → AC18

```rust
// <default><geom shellinertia="true"/></default>, sphere r=1 density=1000.
// Shell mass = ρ × 4πr² = 12566.37 (not solid 4188.79).
let mjcf = r#"<mujoco>
  <default><geom shellinertia="true"/></default>
  <worldbody><body>
    <geom type="sphere" size="1" density="1000"/>
  </body></worldbody>
</mujoco>"#;
let model = load_model(mjcf).unwrap();
assert_relative_eq!(model.body_mass[1], 12566.37, epsilon = 0.1);
```

### Supplementary Tests

| Test | What it covers | Rationale |
|------|---------------|-----------|
| T16 (degenerate mesh shell) | Zero-area mesh with shell mode | Tests fallback path in `compute_mesh_inertia_shell()` |
| T17 (mixed shell/solid body) | Body with one solid sphere + one shell sphere | Tests mass aggregation when geoms have different inertia modes |

---

## Risk & Blast Radius

### Behavioral Changes

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| Default mesh inertia mode | Always Exact | Convex (default) | **Toward MuJoCo** | Models with non-convex meshes and no explicit `inertia` attribute | Set `inertia="exact"` on `<mesh>` to preserve old behavior |
| Ellipsoid volume in `compute_geom_mass()` | Falls to `_ => 0.001` | `4/3 π a b c` | **Toward MuJoCo** | Density-based ellipsoid geoms without explicit `mass` | None — transparent fix (correct values now) |
| `shellinertia` attribute accepted | Not parsed (ignored) | Parsed and applied | **Toward MuJoCo** | New feature — no existing users affected | None — new attribute |
| `inertia` attribute on `<mesh>` | Not parsed | Parsed (4 modes) | **Toward MuJoCo** | New feature | None |
| `exactmeshinertia` emits warning | Silently parsed | Parsed + warning | Neutral | Models using deprecated attribute | Remove attribute (has no effect) |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `sim/L0/mjcf/src/types.rs` | `MeshInertia` enum, `inertia` on `MjcfMesh`/`MjcfMeshDefaults`, `shellinertia` on `MjcfGeom`/`MjcfGeomDefaults`, update defaults | +30 / ~8 modified |
| `sim/L0/mjcf/src/parser.rs` | Parse `inertia` on `<mesh>`, `shellinertia` on `<geom>`, `exactmeshinertia` warning | +30 / ~3 modified |
| `sim/L0/mjcf/src/defaults.rs` | `apply_to_mesh()` inertia cascade, `apply_to_geom()` shellinertia cascade, merge functions | +6 / ~4 modified |
| `sim/L0/mjcf/src/builder/mesh.rs` | `compute_mesh_inertia_shell()`, `compute_mesh_inertia_legacy()`, `compute_mesh_inertia_on_hull()`, `compute_inertia_from_verts_faces()`, `compute_mesh_inertia_by_mode()` | +180 |
| `sim/L0/mjcf/src/builder/geom.rs` | Shell mass/inertia functions, Ellipsoid volume fix, shellinertia validation, shell primitive formulas, GL quadrature | +250 |
| `sim/L0/mjcf/src/builder/mass.rs` | `mesh_inertia_modes` parameter on `compute_inertia_from_geoms()` | ~8 modified |
| `sim/L0/mjcf/src/builder/mod.rs` (or body.rs) | `mesh_inertia_modes` builder field, populate in `process_mesh()`, thread to mass computation | +5 / ~3 modified |
| Test file(s) | New tests T1–T19 | +450 |

### Existing Test Impact

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| `ac1_parser_default_false` | `exactmeshinertia.rs:32` | Pass (unchanged) | Tests `exactmeshinertia` parsing, not mesh inertia mode |
| `ac2a_parser_true` | `exactmeshinertia.rs:54` | Pass + warning | `exactmeshinertia="true"` now emits deprecation warning; model still loads |
| `ac3_unit_cube_mass_inertia` | `exactmeshinertia.rs:88` | Pass (unchanged) | Uses convex cube; Convex == Exact |
| `ac4_asymmetric_mesh` | `exactmeshinertia.rs:142` | Pass (unchanged) | Uses explicit `exactmeshinertia="true"` but algorithm is unchanged for convex meshes |
| All other `exactmeshinertia.rs` tests | `exactmeshinertia.rs` | Pass (unchanged) | All use convex meshes; Convex == Exact numerically |
| Struct literal tests in `mesh.rs` | `mesh.rs:754,779,795,824` | **Compile error** | Must add `inertia: None` to `MjcfMesh` struct literals |
| Struct literal test in `geom.rs` | `geom.rs:535-559` | **Compile error** | Must add `shellinertia: None` to `MjcfGeom` struct literal |

### Non-Modification Sites

| File:line | What it does | Why NOT modified |
|-----------|-------------|-----------------|
| `sim/L0/core/src/convex_hull.rs` | Spec A's `ConvexHull` struct | Read-only consumer; Spec B calls `mesh.convex_hull()` |
| `sim/L0/core/src/mesh.rs:285-293` | `compute_convex_hull()` / `convex_hull()` | Already complete from Spec A |
| `sim/L0/core/src/collision_shape.rs` | `CollisionShape::ConvexMesh` | Collision shapes; not related to inertia |
| `sim/L0/mjcf/src/builder/body.rs` | Body processing | Unchanged; calls `compute_inertia_from_geoms()` which gets new parameter |

---

## Execution Order

1. **S1** (types) — enum + field additions. No behavioral change. Compile and fix struct literals. → verify compilation
2. **S2** (parser) — parse `inertia` on `<mesh>`, `shellinertia` on `<geom>`, `exactmeshinertia` warning → verify T14
3. **S3** (defaults) — cascade `inertia` and `shellinertia` through default classes → verify T12, T19
4. **S8a** (Ellipsoid volume fix) — fix `compute_geom_mass()` Ellipsoid case → verify T13
5. **S4** (shell mesh inertia) — `compute_mesh_inertia_shell()` → verify T2, T3, T16
6. **S5** (legacy mesh inertia) — `compute_mesh_inertia_legacy()` → verify T10
7. **S6** (convex mesh inertia on hull) — `compute_mesh_inertia_on_hull()` → verify T1, T9
8. **S8b-e** (dispatch integration) — mode threading, `compute_mesh_inertia_by_mode()`, `mesh_inertia_modes` parameter, negative volume validation for exact mode → verify T1 regression, T18
9. **S7** (primitive shell formulas) — sphere, box, cylinder, capsule, ellipsoid → verify T4–T8
10. **S9** (shellinertia rejection on mesh) → verify T11
11. **T15, T18** — run full `exactmeshinertia.rs` regression suite + misoriented mesh rejection → verify all 10 existing tests pass + T18

---

## Out of Scope

- **Deeply concave mesh for legacy vs exact demonstration** — EGT-12 notes that the L-shape mesh doesn't demonstrate legacy overcounting (centroid inside solid). A C-shape or U-shape mesh would be needed. Deferred — the algorithm is correct per code review; a more dramatic test mesh is nice-to-have.

- **Flex inertia** — deformable body inertia is a separate subsystem (Phase 10). Not affected by mesh inertia modes.

- **GPU-accelerated inertia computation** — Post-v1.0 performance optimization. No conformance impact.

- **`<compiler exactmeshinertia>` full removal** — CortenForge keeps parsing it for backward compat. Full removal (matching MuJoCo 3.5.0's schema rejection) could be a future cleanup. No conformance impact since the attribute has no behavioral effect.
