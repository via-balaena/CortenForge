# mesh-sdf — Oracle Decomposition Spec (D) + cf-scan-prep Winding Fix (B)

**Status**: SPEC v2. **D.1 SHIPPED 2026-05-18 on dev `9b6863f6` + cold-read polish `8500da57`** (traits + parry oracles + cf-design blanket adapter + deprecated SignedDistanceField wrapper). **D.2 SHIPPED 2026-05-18 on dev `a9de1e62` + cold-read polish `ba0d1106`** (FloodFillSign + CachedGridSdf + 4 synthetic fixtures + numerical-equivalence test; +10 mesh-sdf tests, grade A, WASM clean). **D.3 SHIPPED 2026-05-18 on dev `9ec7462f` + cold-read polish `9ed00337`** — bundled D.3a + D.3b in one commit (181 insertions / 360 deletions across `sdf_layers.rs`, `insertion_sim.rs`, plus a one-line mesh-offset `#![allow(deprecated)]` shim deferring D.4a's deprecation cleanup); polish addressed 3 doc lies + 1 sign-oracle warning + 1 with-caps storage-contract test + cost-figure refresh + banked perf followup. 139 cf-device-design tests pass on release (was 138 — +1 from polish); xtask grade A; clippy + WASM clean. **D.4a SHIPPED 2026-05-18 on dev `9b5d6d7e`** — mesh-offset migrated to `Signed<TriMeshDistance, {FloodFillSign | PseudoNormalSign}>` with the public `SignOracle` enum (default `FloodFill`); the D.3 `#![allow(deprecated)]` shim is GONE; +2 tests (29→31), grade A (coverage 96.7% A+), clippy + WASM clean. **D.4b SHIPPED 2026-05-18 on dev `66826439`** — mesh-lattice's `inner_offset` cavity SDF migrated to `flood_filled_sdf` and the sign-flip negation in the `shape_sdf` closure removed (FloodFill already in the cavity-SDF convention); `build_lattice_to_shell_connections` signature parameterized over `Signed<D, S>`; 97 tests unchanged; grade A (coverage 93.3% A+, up from 84.5% baseline); clippy + WASM clean. Departure from spec §6 #6: no opt-in `PseudoNormalSign` knob added in mesh-lattice (would need a coupled sign-convention branch and no consumer has surfaced the need; the mesh-offset `SignOracle` template makes it cheap to add later).

**Trigger**: The parry-accel arc ([[project-mesh-sdf-parry-accel-spec]], dev `1b333cd4`) unblocked cf-cast-cli geometry-gen but exposed `NotWatertight: 3156 open edges` on `plug_layer_0.stl`. Forensic analysis (this session) traced the failure to parry's `is_inside` returning wrongly-negative far-field at +Y and +Z probes on the iter-1 cleaned scan, propagating through `body.subtract(rind)` into MC, which then extracts a "phantom interior" surface that exits the SDF grid on five of six faces.

**Architectural thesis**: `SignedDistanceField` conflates two orthogonal oracles — an unsigned-distance oracle (parry BVH, reliable) and a sign oracle (parry pseudo-normal, reliable *for some meshes*). The conflation is the bug source: callers cannot tell from the API whether they need defense, so they sometimes forget (cf-cast-cli). Three callers of mesh-sdf have implemented three different sign-defense postures already; the next caller will write a fourth. The right fix is at the architecture layer: split the traits, make composition explicit, and provide first-class `Sign` oracles that compose with any `UnsignedDistance` source.

**Blocks**:
- Workshop iter-1 physical cast (need cf-cast-cli to emit a watertight plug).
- All future mesh-sdf consumers that need reliable sign without rolling bespoke flood-fill.
- F1 ("drop flood-fill robust-sign in cf-device-design") from the parry-accel spec — **CANCELLED**: flood-fill is structurally load-bearing, not just defense-in-depth.

**Parent context**: this arc lands AFTER the parry-accel arc (`1b333cd4`). All forensic evidence and reasoning is in [[project-cf-cast-plug-layer-0-watertight-discovery]] and this session's transcript.

---

## What changed since the parry-accel spec

The parry-accel spec §2 promised:

| Mesh type | Post-parry `distance()` sign |
|---|---|
| Watertight (cleaned scan) | **Reliable** via `TriMesh::project_local_point().is_inside` (pseudonormal-based) |

That promise **does not hold** for iter-1 sock_over_capsule.cleaned.stl. Far-field probes (this session, 2026-05-17 LATE):

```
+x_far (0.080, 0, 0.030)   sdf = +0.046 m  ✓ correct (outside)
-x_far (-0.080, 0, 0.030)  sdf = +0.051 m  ✓
+y_far (0, +0.080, 0.030)  sdf = -0.045 m  ✗ WRONG (claiming inside 80 mm out)
-y_far (0, -0.080, 0.030)  sdf = +0.049 m  ✓
+z_far (0, 0, +0.115)      sdf = -0.039 m  ✗ WRONG (claiming inside 43 mm above dome)
-z_far (0, 0, -0.090)      sdf = +0.036 m  ✓ correct (below cap plane)
origin (0, 0, +0.030)      sdf = -0.026 m  ✓ correct (inside cavity)
```

Asymmetric: wrong on +Y / +Z, correct on -Y / -X / +X / -Z. The asymmetry implicates the dome (high +Z) and one lateral face (high +Y) — places where parry's pseudo-normal aggregation is degenerate, plausibly because (a) the cleaned scan's cap-fan auto-cap inverts CCW winding → inward cap normals (existing memory `cf_scan_prep_cap_winding_concern`) and (b) the dome apex is a high-valence vertex whose angle-weighted normal sum is ill-conditioned.

cf-device-design's preview did NOT see this, because B (`76ccb2e8`, the parry-accel arc's B fix) installed flood-fill robust sign in `sdf_layers::build_cached_scan_sdf`. cf-cast-cli wraps `SharedScanSdf` (parry-direct) via `Arc<dyn Sdf>` with no equivalent defense.

The parry-accel spec was right about the *infrastructure* (parry's BVH is the correct backbone) and wrong about the *sign reliability claim*. This arc corrects the architecture so the wrong claim becomes unrepresentable.

---

## Headline construction

Split `SignedDistanceField` into two traits + a composition struct, AND ship two cache primitives that differ in what they store:

```rust
// mesh-sdf, Layer 0

pub trait UnsignedDistance: Sync + Send {
    fn distance(&self, p: Point3<f64>) -> f64;
    fn closest_point(&self, p: Point3<f64>) -> Point3<f64>;
}

pub trait Sign: Sync + Send {
    /// True iff `p` is inside the closed body. Implementations differ in
    /// (a) what "inside" means for non-manifold input and (b) cost.
    /// Boundary-point behavior is implementation-defined; do not probe
    /// exactly on the surface.
    fn is_inside(&self, p: Point3<f64>) -> bool;
}

#[derive(Debug, Clone)]
pub struct Signed<D: UnsignedDistance, S: Sign> {
    pub distance: D,
    pub sign: S,
}

impl<D: UnsignedDistance, S: Sign> Signed<D, S> {
    pub fn evaluate(&self, p: Point3<f64>) -> f64 {
        let u = self.distance.distance(p);
        if self.sign.is_inside(p) { -u } else { u }
    }
    pub fn distance(&self, p: Point3<f64>) -> f64 { self.evaluate(p) }
    pub fn unsigned_distance(&self, p: Point3<f64>) -> f64 { self.distance.distance(p) }
    pub fn is_inside(&self, p: Point3<f64>) -> bool { self.sign.is_inside(p) }
    pub fn closest_point(&self, p: Point3<f64>) -> Point3<f64> { self.distance.closest_point(p) }
}
```

Concrete oracles shipped in D.1 + D.2:

```rust
// Unsigned-distance oracle: parry BVH-backed TriMesh.
pub struct TriMeshDistance { tri_mesh: Arc<parry3d::shape::TriMesh>, mesh: IndexedMesh }
impl UnsignedDistance for TriMeshDistance { ... }

// Sign oracles: pick at composition time.
pub struct PseudoNormalSign { tri_mesh: Arc<parry3d::shape::TriMesh> }  // fast, fragile on cleaned scans
impl Sign for PseudoNormalSign { ... }

/// Sign-only cache. Composes with any UnsignedDistance. Cheap (one bool
/// per grid cell + an inflated "wall band" treated as a barrier so the
/// flood can't leak across the surface). Use this when distance
/// precision matters and you only need to fix the sign.
pub struct FloodFillSign {
    region: Vec<Region>,                 // Inside / Outside / Wall per cell
    origin: Point3<f64>,
    cell_size: f64,
    dims: [usize; 3],
}
impl FloodFillSign {
    pub fn build<D: UnsignedDistance + ?Sized>(
        distance: &D,
        bounds: Aabb,
        cell_size: f64,
        wall_threshold_factor: f64, // 0.75 matches cf-device-design's shipped value
    ) -> Result<(Self, FloodFillReport), FloodFillError> { ... }
}
impl Sign for FloodFillSign { ... }
```

A second primitive caches BOTH signed distance and sign in a grid — the migration target for cf-device-design's two existing full-grid SDF caches:

```rust
/// Full signed-distance grid. Owns: unsigned-distance grid (filled once
/// from the supplied UnsignedDistance) + the FloodFillSign output applied
/// to the stored magnitudes → signed values per cell. Queries are
/// trilinear interpolation, O(1) per query.
///
/// Use this when the same SDF is queried many times (preview SDF re-used
/// across frames; insertion-sim ScalarGrid sampling at every BCC tet
/// node). For one-shot use cases (cf-cast-cli's MC sweep), prefer
/// `Signed<TriMeshDistance, FloodFillSign>` — distance precision stays
/// exact at the cost of one parry query per cell corner.
pub struct CachedGridSdf {
    grid: ScalarGrid,                  // signed distance per cell (post-flood)
    sign: FloodFillSign,               // retained for is_inside queries outside the cell-center lattice
}
impl CachedGridSdf {
    pub fn build<D: UnsignedDistance + ?Sized>(
        distance: &D,
        bounds: Aabb,
        cell_size: f64,
        wall_threshold_factor: f64,
    ) -> Result<(Self, FloodFillReport), FloodFillError> { ... }
}
impl UnsignedDistance for CachedGridSdf { /* trilinear |grid| */ }
impl Sign for CachedGridSdf { /* via stored grid sign or FloodFillSign */ }
// And, in cf-design (cross-crate orphan-rule reason):
// impl cf_design::Sdf for CachedGridSdf { eval, grad via finite diff }
```

Notes:
- `Region` is the same three-state enum (Inside / Outside / Wall) that the existing cf-device-design + insertion_sim flood-fills use. Imported by `FloodFillSign`.
- The wall band exists because a binary inside/outside flood can leak across the surface between adjacent lattice points whose midpoint crosses a face. The wall band guarantees 6-connectivity watertightness: any face crossing puts at least one adjacent lattice point inside the wall. After the flood, a multi-source BFS expands Outside/Inside labels into the wall cells (each wall cell takes its nearest non-wall label).
- `FloodFillReport` exposes the same diagnostics cf-device-design's `GridSdfReport` does: cell counts per region, inside-component count (1 = healthy, >1 = flood leak), build time.
- `TriMeshDistance` and `PseudoNormalSign` both want a `TriMesh`; share via `Arc` so the `Signed<TriMeshDistance, PseudoNormalSign>` composition owns one BVH, not two.

---

## High-conviction decisions

### 1. Trait split: orthogonal `UnsignedDistance` + `Sign`

**Pick**: split. Distance and sign are different oracles with different cost / reliability tradeoffs; conflating them is the bug source.

**Rejected**:
- **Single `Sdf` trait, `Sign` as associated type**. Cleaner type-level but forces every consumer to name the sign type explicitly even when they don't care.
- **`SignedDistanceField` with a `sign_oracle` knob (option C from this session)**. Papers over the conflation; the type still pretends to be a single oracle and the wrong code remains writable.
- **`UnsignedSdf` and `SignedSdf` as different types with explicit conversion**. Most type-safe but requires cf-design's `Sdf` trait to also split, cascading. Too radical.

**Side effect**: `Signed<D, S>` implements cf-design's `Sdf` trait via its `evaluate` method, so existing consumers (`Solid::from_sdf`) accept it transparently. **cf-design's `Sdf` trait stays unchanged.**

### 2. `Sign::is_inside` returns `bool`, not `Option<bool>`

**Pick**: `bool`. Consumers always have to pick a sign at some point; surfacing tri-state propagates upward forever. Boundary behavior is documented as implementation-defined (matches the existing convention).

**Rejected**:
- `Option<bool>` + composable fallback. More honest but every `Signed::evaluate` becomes a layered match. Cost not worth the rare benefit.

### 3. Sign oracles + cache primitives shipped in v1

**Pick**: two `Sign` oracles + two cache primitives, all promoted from existing implementations.

`Sign` oracles:
- `PseudoNormalSign` wraps parry's `TriMesh::project_local_point(p, false).is_inside`. Matches current `SignedDistanceField` behavior. Fast, fragile on cleaned scans, **fine on synthetic / well-formed meshes**.
- `FloodFillSign` ports the existing 3-region flood (Inside / Outside / Wall) from `cf-device-design::sdf_layers::build_cached_scan_sdf` and `cf-device-design::insertion_sim::build_grid_sdf` (they are two near-identical copies of the same algorithm). Wall band defaults to `0.75 × cell_size`. Robust for any mesh whose interior is connected AND fits inside the supplied bounds with the wall band below grid resolution.

Cache primitives:
- `FloodFillSign` (above) — sign-only cache. Composes with any `UnsignedDistance` source.
- `CachedGridSdf` — full signed-distance grid. Owns both the unsigned-distance grid and the FloodFillSign. Trilinear-interpolated queries; O(1) per lookup. Migration target for cf-device-design's two existing full-grid caches.

**Banked**:
- `WindingNumberSign` (generalized winding number, Jacobson et al). Slow but works on degenerate / open / soup meshes. Wait for an actual use case.
- `RayCastSign` (Jordan-curve via +X ray-parity). The old pre-parry sign source. No use case to bring back.

### 4. Migration of `SignedDistanceField`

**Pick**: keep `SignedDistanceField` as a deprecated convenience constructor that returns `Signed<TriMeshDistance, PseudoNormalSign>` with an Arc-shared TriMesh internally. Add `#[deprecated]` with a pointer to the new API. Hard removal in a follow-up release.

**Migration cost**: every consumer needs ONE construction-site edit (`SignedDistanceField::new(mesh)?` → explicit composition with a chosen sign oracle). No call-site changes for `distance` / `unsigned_distance` / `is_inside` / `closest_point` / `mesh`. Standalone `signed_distance` + `unsigned_distance` free functions are deleted (their behavior is trivially expressed by the new composition).

```rust
#[deprecated(note = "Conflates distance and sign. Pick a sign oracle explicitly. \
                     Existing call sites: SignedDistanceField::new(mesh) is equivalent to \
                     Signed { distance: TriMeshDistance::new(mesh)?, \
                              sign: PseudoNormalSign::from(&distance) } \
                     but PseudoNormalSign is unreliable on cf-scan-prep cleaned scans \
                     (see docs/MESH_SDF_ORACLE_DECOMPOSITION_SPEC.md and project memory \
                     `pinned-floor-visual-gate-postmortem`). Prefer FloodFillSign for any \
                     SDF derived from a body-part scan.")]
pub fn new(mesh: IndexedMesh) -> SdfResult<Signed<TriMeshDistance, PseudoNormalSign>> { ... }
```

### 5. `FloodFillSign` construction: takes any `UnsignedDistance` + bounds + cell_size

**Pick**: parametric over the distance oracle. Lets `FloodFillSign` work over a TriMesh distance, a cached-grid distance, or any future oracle. Construction cost is one grid build + one BFS pass; happens once at cache time.

```rust
impl FloodFillSign {
    pub fn build<D: UnsignedDistance + ?Sized>(
        distance: &D,
        bounds: Aabb,
        cell_size: f64,
    ) -> Self {
        // 1. Allocate ScalarGrid<bool> over `bounds` at `cell_size`.
        // 2. Evaluate distance at every cell corner; tag cells "outside" if
        //    the signed distance is unambiguously positive (heuristic: any
        //    corner is positive AND no corner is below -cell_size).
        // 3. Seed BFS from the 8 corner cells (assumed outside).
        // 4. BFS over the "outside" cells across face-adjacent neighbors
        //    where the SDF is positive (i.e., neighbor is also outside).
        // 5. Any unreached cell is "inside."
        // 6. Store the bool grid + origin + cell_size for is_inside queries.
    }
}
```

`is_inside(p)` looks up the cell containing `p` and returns the stored bool. Queries outside the build bounds are documented as "outside" (defensive default).

### 6. Per-consumer migration scope (post stress-test)

The stress-test grep surfaced **more consumers than v1 of the spec assumed**. Updated list:

| # | Site | Disposition |
|---|------|-------------|
| 1 | `cf-device-design::sdf_layers::build_cached_scan_sdf` | Migrate to `CachedGridSdf::build` (it caches the entire signed grid today, not just a sign mask). Drop the bespoke 3-region + dual-BFS code (~150 LOC). |
| 2 | `cf-device-design::insertion_sim::build_grid_sdf` returning `GridSdf` | Migrate to `CachedGridSdf::build` and replace the in-crate `pub struct GridSdf { grid: SdfGrid }` with a thin newtype OR drop in favor of `CachedGridSdf` directly. ~150 LOC. **Keep** the `GridSdfReport` diagnostic interface — promote it to `FloodFillReport` in mesh-sdf so insertion_sim's connected-component health check still surfaces. |
| 3 | `cf-cast-cli::derive::derive_spec_and_ribbon` (closed scan SDF) | NEW defense: build `FloodFillSign` once over the scan AABB + cumulative-thickness margin, wrap `closed_sdf_arc` in `Signed<TriMeshDistance, FloodFillSign>` (the `Arc<dyn Sdf>` pattern still works because `Signed` impls `Sdf` via cf-design). |
| 4 | `cf-cast-cli::derive::derive_spec_and_ribbon` (open cap-stripped SDF) | Wrapped by `UnsignedRindSdf` which consumes `.abs()` only. Migrate to bare `TriMeshDistance` (sign-immune consumer). |
| 5 | `mesh-offset::offset_mesh` (`SignedDistanceField::new(mesh.clone())` then `sdf.distance(p)` per-cell) | NEW vulnerability: same shape as cf-cast-cli's mesher. Currently fed by user-supplied or intermediate meshes that may have the same sign issues. Migrate to `Signed<TriMeshDistance, FloodFillSign>` by default; optional `OffsetConfig::sign_oracle` if a caller has a known-good mesh and wants the fast path. |
| 6 | `mesh-lattice::infill` (`Arc::new(SignedDistanceField::new(inner_offset))`) | Comment at infill.rs:420-428 already documents that `point_in_mesh`'s ray-cast was ~36% wrong and the face-normal heuristic is "stable under MC discretization." Same migration as mesh-offset: prefer `FloodFillSign` by default, opt-in for `PseudoNormalSign` fast path. |
| 7 | `cf-design::sdf` (the `impl Sdf for SignedDistanceField`) | This lives in cf-design today (orphan-rule lives below). For D.1 the new impl is `impl<D: UnsignedDistance + 'static, S: Sign + 'static> Sdf for Signed<D, S>` in cf-design. Tests in `solid_layered.rs` move from `SignedDistanceField::new(...)` to explicit composition. |
| 8 | `mesh/mesh/tests/api_regression.rs` | Mesh facade test. Trivial migration — keep one regression test that exercises the deprecated path. |
| 9 | `cf-design::sdf::tests` | Internal tests of the `Sdf for SignedDistanceField` impl. Migrate to test `Sdf for Signed<D, S>` instead. |
| 10 | `cf-cap-planes::lib.rs`, `cf-cast::ribbon.rs` | Doc-only references to `SignedDistanceField`. Update docstrings, no functional change. |

**Cross-crate orphan rule**: `Sdf` is owned by cf-design; `Signed<D, S>` is owned by mesh-sdf. The `impl Sdf for Signed<D, S>` MUST live in cf-design (where the trait lives), mirroring the existing `impl Sdf for SignedDistanceField` at `cf-design/src/sdf.rs:111`.

### 7. `Sdf` trait (cf-design) interaction

cf-design's `Sdf` trait is the canonical SDF interface in the `Solid` AST. Its full definition lives at `design/cf-design/src/sdf.rs:28`:

```rust
pub trait Sdf: Send + Sync {
    fn eval(&self, p: Point3<f64>) -> f64;
    fn grad(&self, p: Point3<f64>) -> Vector3<f64>;
}
```

with blanket impls for `Box<T>`, `Arc<T>`, `Solid`, and (currently) `SignedDistanceField`. The blanket `Arc<T>` impl is what the parry-accel arc relied on for cf-cast-cli's `Arc<dyn Sdf>` clone-shared pattern; the new types must satisfy the same lifetime + trait-object discipline.

For D.1:

```rust
// In design/cf-design/src/sdf.rs:
impl<D, S> Sdf for mesh_sdf::Signed<D, S>
where
    D: mesh_sdf::UnsignedDistance + Send + Sync + 'static,
    S: mesh_sdf::Sign + Send + Sync + 'static,
{
    fn eval(&self, p: Point3<f64>) -> f64 { self.evaluate(p) }
    fn grad(&self, p: Point3<f64>) -> Vector3<f64> {
        // Central finite difference, eps = 1e-6 (matches existing impl).
        let eps = 1e-6;
        let inv_2eps = 0.5 / eps;
        Vector3::new(
            (self.evaluate(Point3::new(p.x + eps, p.y, p.z)) -
             self.evaluate(Point3::new(p.x - eps, p.y, p.z))) * inv_2eps,
            (self.evaluate(Point3::new(p.x, p.y + eps, p.z)) -
             self.evaluate(Point3::new(p.x, p.y - eps, p.z))) * inv_2eps,
            (self.evaluate(Point3::new(p.x, p.y, p.z + eps)) -
             self.evaluate(Point3::new(p.x, p.y, p.z - eps))) * inv_2eps,
        )
    }
}

// CachedGridSdf also impls Sdf, using its native trilinear gradient
// instead of central differences (the grid IS the smoothed source).
impl Sdf for mesh_sdf::CachedGridSdf { /* via cached grid */ }
```

The `+ 'static` is needed because `Solid::from_sdf<S: Sdf + 'static>` (verified at `design/cf-design/src/solid.rs:1105`). The existing `impl Sdf for Arc<T>` blanket means `Arc<Signed<D, S>>` still satisfies `Sdf` so cf-cast-cli's shared-clone pattern survives.

**The existing `impl Sdf for SignedDistanceField` (cf-design/src/sdf.rs:111) gets a `#[deprecated]` annotation pointing at `Signed<TriMeshDistance, PseudoNormalSign>`.** Its docstring (lines 102-110) already warned about the sign-heuristic failure mode that bit us — the architecture fix makes that warning unnecessary by making the choice explicit.

### 8. WASM compat: no regression

`FloodFillSign`'s grid + BFS are pure-Rust (no SIMD, no rayon). `TriMeshDistance` + `PseudoNormalSign` are already wasm-clean per the parry-accel arc's WASM gate. D adds no new WASM risk. P1 gate: re-run `cargo build -p mesh-sdf --target wasm32-unknown-unknown` after each sub-leaf.

### 9. Test fixtures: load-bearing for B's gate

D.2 ships synthetic fixtures that mimic our real failure modes. These become the **automated gate for B's success** (B claims to fix cf-scan-prep's output; the fixtures let us verify that `PseudoNormalSign` returns correct signs on the fixed geometry without needing a re-prepped iter-1 scan loop).

Fixtures (all `IndexedMesh`):
- `closed_pyramid` — already exists in mesh-sdf tests; reused as the "well-formed" sanity case.
- `dome_with_inward_cap` — synthetic mimic of cf-scan-prep auto-capped output. Closed prism with cap-fan whose triangles have inverted CCW winding (normals pointing INTO the body). Should produce wrong-sign far-field with `PseudoNormalSign`; correct sign with `FloodFillSign`.
- `dome_with_outward_cap` — same shape but with correct CCW winding (normals OUTWARD). Should produce correct sign with both oracles. **Post-B, the previously-failing case should match this one.**
- `high_valence_dome_apex` — sphere-like geometry with a single high-valence vertex (e.g., 32 incident triangles). Tests pseudo-normal degeneracy at near-singular vertices.

### 10. Stress-test against the code (this session, before D.1)

**Mandatory before P1 of any sub-leaf**:
- Read cf-design's `Sdf` trait definition + `Solid::from_sdf` signature. Confirm `impl Sdf for Signed<D, S>` works against the actual trait.
- Read `SharedScanSdf` in cf-cast-cli. Confirm migration is a one-line edit at the construction site.
- Read `sdf_layers::build_cached_scan_sdf` + insertion_sim's `build_grid_sdf`. Confirm the bespoke flood-fill semantics match what `FloodFillSign::build` would produce. If they differ, document the difference + decide which is canonical before D.2.
- Grep for ALL `SignedDistanceField` consumers in-tree. Document the migration cost for each.
- Read cf-scan-prep's `auto_cap_open_boundaries` to confirm the inward-winding hypothesis (B's premise).

The stress-test pass may reveal that one or more decisions above need revision. Iterate the spec before writing code.

---

## Sub-leaf ladder

### D — Oracle decomposition (in-arc)

**D.1 — Traits + parry oracles + cf-design adapter + deprecated wrapper.** One commit if the impls compose cleanly; small ladder otherwise.
- Add `UnsignedDistance` + `Sign` traits + `Signed<D, S>` struct + `Region` enum + `FloodFillError` + `FloodFillReport` to mesh-sdf.
- Add `TriMeshDistance` + `PseudoNormalSign` impls (parry-backed, both sharing an `Arc<TriMesh>`).
- Add `impl<D, S> Sdf for Signed<D, S>` in cf-design (NOT mesh-sdf — orphan rule).
- Deprecate `SignedDistanceField::new` (becomes a convenience constructor returning `Signed<TriMeshDistance, PseudoNormalSign>`).
- Delete standalone `mesh_sdf::signed_distance` + `mesh_sdf::unsigned_distance` free functions (trivially expressible via the composition).
- All existing mesh-sdf tests pass with minimal edits.
- WASM gate.
- `cargo run -p xtask -- grade mesh-sdf` ≥ A.

**D.2 — `FloodFillSign` + `CachedGridSdf` + synthetic fixtures.** SHIPPED 2026-05-18 dev `a9de1e62`. One commit per spec (BFS + 3-region machinery + both primitives + 4 fixtures + numerical-equivalence test all landed together). +10 mesh-sdf tests (18 → 28 + 1 doctest); WASM clean; clippy clean; grade automated A; coverage 93.2% A+; downstream cf-design / cf-device-design / cf-cast-cli build unchanged. Helper `flood_filled_sdf(mesh, bounds, cell_size, wall_threshold_factor)` returns the recommended `Signed<TriMeshDistance, FloodFillSign>` composition in one call (for D.5's cf-cast-cli ergonomics). Private `SdfGrid` storage local to mesh-sdf (deferred the public `ScalarGrid` movement out of D.2 scope — see "Implementation deviations" §A below).
- Port the 3-region flood-fill from `cf-device-design::sdf_layers::build_cached_scan_sdf` into mesh-sdf. Generic over any `UnsignedDistance`.
- `FloodFillSign::build(distance, bounds, cell_size, wall_threshold_factor) -> Result<(Self, FloodFillReport), FloodFillError>`.
- `CachedGridSdf::build(...)` — same signature, fills both an unsigned grid AND the FloodFillSign; stores the signed grid for O(1) queries.
- `impl Sign for FloodFillSign`, `impl UnsignedDistance + Sign for CachedGridSdf`, `impl Sdf for CachedGridSdf` (in cf-design).
- 4 synthetic fixtures + tests:
  - `closed_pyramid` — sanity, both oracles correct.
  - `dome_with_inward_cap` — cap-fan with inverted CCW winding (mimic of pre-B cf-scan-prep output). `PseudoNormalSign` must produce wrong sign at far-field +Z probe; `FloodFillSign` must produce correct sign. **Load-bearing for B's gate.**
  - `dome_with_outward_cap` — same shape, correct winding. Both oracles agree. **Post-B target for cleaned-scan output.**
  - `high_valence_dome_apex` — sphere-cap with single 32-fan vertex at apex. Exercises pseudo-normal degeneracy at the apex.
- `FloodFillReport::inside_components == 1` invariant on every fixture (flood-leak detector).
- WASM gate.
- `cargo run -p xtask -- grade mesh-sdf` ≥ A.

**D.3 — cf-device-design migration.** SHIPPED 2026-05-18 dev `9ec7462f`. Spec called for two commits (D.3a + D.3b); bundled into one because the pre-commit hook's `-D warnings` clippy run on cf-device-design propagates rustflags through the transitive mesh-offset dep (whose D.4-pending `SignedDistanceField` deprecations would fail D.3a alone). User-confirmed bundling decision before commit per [[feedback-autonomous-architecture]].
- D.3a: `sdf_layers::build_cached_scan_sdf` adopts `CachedGridSdf::build` — Region/neighbours6/wall-threshold constant deleted, 4-pass body replaced with one `CachedGridSdf::build` call + unpack via `closed_grid.position()` trilinear sample (1e-12 ulp-bounded). 88 inserted / 196 deleted. `CachedScanSdf` public surface byte-preserved; `sdf_closed` + `sdf_open` retyped to explicit `Arc<Signed<TriMeshDistance, PseudoNormalSign>>` (= the deprecated `SignedDistanceField` alias) to drop the deprecation warning without touching the heat-map closest-point projection downstream.
- D.3b: `insertion_sim::build_grid_sdf` adopts `CachedGridSdf::build`. Duplicate Region/neighbours6/4-pass body deleted; `GRID_SDF_SMOOTH_SIGMA_CELLS` + `gaussian_smooth_3d_separable` KEPT (spec deviation §C — mesh-sdf primitives smoothing-free; Gaussian re-applied externally as FEM-specific contact-gradient C¹ approximation). `GridSdfReport` projected from `FloodFillReport` (field renames: `n_outside`↔`outside_cells` etc.; `build_ms` from local timer covers both flood-fill + Gaussian). Three other `SignedDistanceField::new` call sites (`run_sdf_bridge_spike` + two cross-check tests) migrated to explicit composition. 82 inserted / 164 deleted.
- mesh-offset shim: 11-line `#![allow(deprecated)]` block at `mesh/mesh-offset/src/offset.rs` head, removed by D.4a alongside its migration. Pure lint deferral — no behavior change.
- Visual gate on iter-1 sock: load-bearing post-commit check (user runs cf-device-design on `~/scans/sock_over_capsule.cleaned.stl` + iter-1 `.prep.toml`, confirms preview matches pre-D.3 state). Spec-internal verification: bit-equivalence test `no_caps_closed_grid_magnitude_equals_unsigned_distance` (`|cached.abs() - sdf_closed.unsigned_distance(p)| < 1e-12` per cell) PASSED; `extract_layer_surface_no_caps_byte_identical_to_pre_pinned_floor` PASSED.
- Insertion-sim: existing 138 cf-device-design tests cover the smoothed `GridSdf` consumers (intruder eval / geometry build / spike); no new bit-equivalence regression test added because the Gaussian smoothing pass downstream washes out sub-ulp differences before any consumer reads them.
- `cargo run -p xtask -- grade cf-device-design`: A (Documentation A, Clippy A, Safety A, Dependencies A; Coverage / Layer / WASM out of scope for the bin profile).

**D.4 — Other mesh-sdf consumers (mesh-offset + mesh-lattice).** Two commits, both SHIPPED 2026-05-18.

- D.4a SHIPPED on dev `9b5d6d7e` (3 files / +121 / −24): `mesh-offset::offset_mesh` adopts `Signed<TriMeshDistance, FloodFillSign>` by default. New public `SignOracle` enum (`FloodFill` (default) | `PseudoNormal`) re-exported from the crate root; `OffsetConfig.sign_oracle` field with `..Self::default()` tail in the preset constructors; `with_sign_oracle` builder; `sample_sdf_to_grid` parameterized over `Signed<D, S>`; `OffsetError::FloodFill(#[from] mesh_sdf::FloodFillError)` + `From<FloodFilledSdfBuildError> for OffsetError` decomposition impl. The 11-line `#![allow(deprecated)]` module-head shim landed in D.3 is DELETED. Tests cover both paths (`offset_cube_pseudo_normal_opt_in` + `offset_config_default_sign_oracle_is_flood_fill`); 31 pass (was 29). Grade A (coverage 96.7% A+, all 7 automated criteria green). Clippy clean (`-D warnings`), WASM clean. Existing OffsetConfig consumers (mesh-shell × 3, mesh-lattice × 1, mesh-offset internal tests + examples) all use builder-style constructors (`OffsetConfig::default().with_resolution(...)`); the new field doesn't break any of them.

- D.4b SHIPPED on dev `66826439` (1 file / +68 / −37): `mesh-lattice::generate_infill`'s `inner_offset` cavity SDF migrated to `flood_filled_sdf(inner_offset.clone(), inner_bounds, offset_resolution, WALL_THRESHOLD_FACTOR_DEFAULT)`. Bounds = `inner_offset.aabb()` inflated by 5 × `offset_resolution` (matches mesh-offset's internal padding). The sign-flip negation in the lattice `shape_sdf` closure is gone — `move |p| inner_sdf_for_clip.distance(p)` instead of `move |p| -inner_sdf_for_clip.distance(p)`. Reason: pseudo-normal sign on the inverted-winding `inner_offset` (`signed_volume() < 0`) read cavity as "outside" the closed surface; FloodFill reads topological reachability (cavity = unreachable from corner seeds → Inside → negative) which is already in the cavity-SDF convention. Bonus correctness gain: the documented pseudo-normal limitation for non-convex inputs (e.g., torus holes where outside-the-part got incorrectly included in the lattice domain) is FIXED — flood-fill labels reachability not closest-face-normal direction. `build_lattice_to_shell_connections` signature parameterized over `Signed<D: UnsignedDistance, S: Sign>` (only `closest_point` consumed; sign type is pass-through). Function docstring drops the stale "BVH-accelerated mesh-sdf is a v0.9 candidate" line (parry-accel ships today). 30-line inline comment block rewritten end-to-end with the ray-cast → pseudo-normal → flood-fill history + why-the-negation-existed + bonus-fix reasoning + pointer to this spec.

  **Spec §6 #6 departure (small)**: no opt-in `PseudoNormalSign` knob on `InfillParams`. The two oracles need DIFFERENT closures (the sign convention flips), so a knob would add a coupled match across the construction site AND the closure. No mesh-lattice consumer has surfaced the need today, the FloodFill cost is a one-time grid build (`O(N_grid)`), and the mesh-offset `SignOracle` template makes it cheap to add later. Bundling the knob now would land ~30 lines of branching for a path nobody exercises.

  Tests + gates: 97 pass (unchanged baseline). `cargo run -p xtask -- grade mesh-lattice`: **A across all 7 automated criteria; coverage UP to 93.3% A+ (was 84.5% A baseline)** — the migration's net code shrink and the new code paths sitting inside the already-exercised `generate_infill` pipeline pushed coverage higher, not lower. Clippy + build + doc + WASM all clean.

**Stress-test finding (banked for D.5/D.6)**: pre-existing breakage from D.1 at `examples/mesh/mesh-sdf-distance-query/src/main.rs:60` — imports four free functions (`closest_point_on_triangle`, `point_in_mesh`, `point_segment_distance_squared`, `ray_triangle_intersect`) that D.1 deleted. `cargo check --workspace` fails on this example today; not introduced by D.4. Other workspace deprecation warnings (cf-cast-cli, examples in cast/layered-silicone, sim-soft/{layered-silicone-device, mesh-scan-as-solid}, mesh/shell-generation-high-quality) all still use `SignedDistanceField`; cf-cast-cli is D.5 scope explicitly, the examples need a follow-up sub-leaf during the D.6 docs pass.

**D.5 — cf-cast-cli migration (workshop unblock).** Single commit.
- `derive_spec_and_ribbon` builds `FloodFillSign` once over the scan AABB + cumulative-thickness margin (reuses the `sdf_bounds` already computed).
- `closed_sdf_arc` becomes `Arc::new(Signed { distance: TriMeshDistance::from(scan_sdf.mesh()), sign: flood_fill_sign })`.
- Open-mesh SDF (used by `UnsignedRindSdf`) drops to `TriMeshDistance`.
- iter-1 mold export completes end-to-end: 4 piece STLs + 2 plug STLs + procedure.md in `~/scans/cast_iter1_design/` in single-digit minutes.
- Visual gate on plug STLs (no open edges, looks like a cavity shape).
- `cargo run -p xtask -- grade cf-cast-cli` ≥ A.

**D.6 — Memory + docs.**
- Spec memo (this doc) annotated SHIPPED with as-built notes + actual LOC counts.
- New patterns memo: "decompose conflated oracles" pattern for future cross-crate redesigns.
- Update parry-accel spec memo: §F1 marked CANCELLED; cite this spec.
- Update `project_cf_cast_plug_layer_0_watertight_discovery.md` memo: RESOLVED.
- MEMORY.md Resume note updated to point at workshop iter-1 physical cast OR B.0 audit (whichever the user prefers next).

### B — cf-scan-prep winding fix (separate arc, follows D)

**Sequenced after D, NOT bundled.** D is a refactor (clean diffs, API reasoning); B is a behavior change (sign flips, regression hunting). Mixing them makes commits harder to reason about.

**B.0 — Audit (pre-implementation gate).**
- List every consumer of cf-scan-prep's cleaned mesh face normals (not just SDF — anything that reads `mesh.faces` and computes a normal).
- For each consumer, document what it does with the normal direction.
- Identify which consumers would silently change behavior if normals flipped.
- Decide per-consumer: (a) needs explicit fix alongside B, (b) tolerates the flip, (c) inverts manually elsewhere and that workaround can now be deleted.

**B.1 — Fix `auto_cap_open_boundaries` winding.**
- Trace the CCW-winding flip site in cf-scan-prep.
- Fix to emit outward-pointing cap normals.
- Add regression test: load a fixture with a known-cap boundary, run `auto_cap_open_boundaries`, assert the cap-fan face normals point outward (away from body centroid).

**B.2 — Validate against D's synthetic fixtures.**
- `dome_with_inward_cap` (D.2 fixture) is the synthetic mimic of pre-B output.
- Build the post-B equivalent: a `dome_with_outward_cap` should be what the fixed code produces.
- Run iter-1 sock re-prepped through post-B cf-scan-prep. Compare `PseudoNormalSign` against `FloodFillSign` on the new cleaned scan; signs should agree everywhere.

**B.3 — cf-cast-cli fast-path opt-in.** Optional follow-up.
- Add a flag (e.g., `[cast.sign_oracle] = "pseudo_normal"`) that lets workshop users skip `FloodFillSign`'s grid build cost.
- Document that the fast path is safe only for cleaned scans produced by post-B cf-scan-prep.
- This is an opt-in performance optimization, NOT a defense rollback.

---

## Open questions (resolve in stress-test pass)

1. **`'static` bound on `Signed::Sdf` impl.** Does cf-design's `Sdf` trait actually require `'static` for `Solid::from_sdf`? If yes, `Signed<D, S>` needs `D: 'static, S: 'static`, which constrains `FloodFillSign` to own its grid (it already does).
2. **`Arc<dyn UnsignedDistance>` and `Arc<dyn Sign>`.** cf-cast-cli's existing pattern wraps `Sdf` in `Arc<dyn Sdf>` so multiple `Solid::from_sdf` calls share the underlying parry mesh. Does `Signed<Arc<dyn UnsignedDistance>, Arc<dyn Sign>>` work, or do we need an alternative composition for the shared case? May require trait-object dyn-safety review.
3. **`FloodFillSign` grid cost on iter-1 scan.** Scan AABB 71×71×127 mm. At 3 mm cells, that's ~24×24×42 = ~24k cells. BFS over 24k cells is trivially fast. At 1 mm cells, ~712k cells — still trivially fast. No concern at workshop scale. Worth documenting the expected cost.
4. **Migration path for downstream consumers we don't know about yet.** Pre-D.1 grep should be comprehensive. Anything outside of cf-device-design + cf-cast-cli + mesh-sdf itself needs a migration note.
5. **Test fixture for `dome_with_inward_cap`**: is "inward winding" sufficient to reproduce parry's failure mode in a 30-vertex synthetic, or does it require additional geometric complexity (like the high-valence dome apex)? If we can't reproduce in a synthetic, B's gate becomes harder to construct.

---

## Fragile bits

1. **`Sdf` trait coupling across crate boundaries.** cf-design's `Sdf` trait is the load-bearing interface for `Solid`. Any change to mesh-sdf's API must keep `Sdf for Signed<D, S>` working transparently. Pin this with a doctest in mesh-sdf.
2. **Grid bounds for `FloodFillSign`.** The flood-fill defines "inside" as "unreached by BFS from outside corners." If the bounds are too tight (don't include enough margin), corner cells might already be inside the body, and the BFS will incorrectly mark the whole grid as inside. Document a minimum-margin rule: bounds must contain a positive-SDF point at every corner. Validate at construction time; bail with a descriptive error.
3. **BFS seed contract.** What if a corner cell is itself inside the body? `build` should bail loudly or auto-expand bounds; silently propagating from one wrong seed corrupts the whole grid.
4. **D and the deprecated `SignedDistanceField` typedef.** The deprecation should fire on every existing call site to surface the migration. Use `#[allow(deprecated)]` on the convenience constructor's *body* (it intentionally uses the old name internally) but not the public path.
5. **Bit-equivalence claim for D.3.** cf-device-design's bespoke flood-fill has subtle implementation choices (8-neighbor vs 6-neighbor BFS, sign-threshold values, corner-cell handling). `FloodFillSign::build` may produce a different bool grid. If so, D.3 needs an explicit "bit-equivalent migration" sub-step that ports the differences (or makes a deliberate choice to diverge with documented rationale).
6. **B's blast radius.** Even with B.0 audit, behavior changes can have non-obvious downstream effects (e.g., a shader that consumes face normals through a different path). Plan for one cycle of cf-device-design visual gates post-B to catch shading regressions.
7. **Naming: `is_inside` vs `inside`.** Current convention is `is_inside`. Keep for consistency, even though it's slightly redundant.

---

## Acceptance criteria

D arc:
- [ ] D.1: traits + parry oracles + deprecated wrapper land; mesh-sdf tests pass; WASM clean; grade A.
- [ ] D.2: `FloodFillSign` + 4 synthetic fixtures land; tests pin sign behavior per fixture; WASM clean; grade A.
- [ ] D.3a + D.3b: cf-device-design migrations land; visual gate on iter-1 sock matches current state; grade A.
- [ ] D.4: cf-cast-cli iter-1 mold export completes end-to-end (6 STLs + procedure.md); visual gate on plug STLs; grade A.
- [ ] D.5: memory + docs updated.
- [ ] WORKSHOP iter-1 PHYSICAL CAST UNBLOCKED.

B arc (separate):
- [ ] B.0: audit complete; per-consumer disposition documented.
- [ ] B.1: winding fix + regression test land in cf-scan-prep; grade A.
- [ ] B.2: D.2 synthetic fixtures with corrected winding pass under `PseudoNormalSign`; iter-1 re-prepped scan agrees between `PseudoNormalSign` and `FloodFillSign`.
- [ ] B.3 (optional): cf-cast-cli sign-oracle flag lands; documented as opt-in fast path.

---

## Followup arcs (out of scope)

- **F (formerly F2 from parry-accel)**: drop `SDF_SOURCE_TARGET_FACES = 2500` decimation in cf-device-design (now cosmetic post-parry).
- **G**: `WindingNumberSign` for non-manifold / soup inputs. No current use case; bank.
- **H**: parry's `cast_ray` could accelerate fit-viz rungs 2-6 (insertion-sim's `validation`). Adjacent arc.
- **I**: hard-remove deprecated `SignedDistanceField` after one release cycle.

---

## Stress-test findings (v1 → v2)

Six findings from grepping the actual code after v1 was drafted. Each forced a spec revision.

**Finding 1: `impl Sdf for SignedDistanceField` lives in cf-design, not mesh-sdf** (`design/cf-design/src/sdf.rs:111`). Orphan-rule constraint: cf-design owns the `Sdf` trait, so the adapter for any mesh-sdf type must live in cf-design.
→ **Revision**: §7 now specifies `impl Sdf for Signed<D, S>` lives in cf-design, not mesh-sdf. D.1 sub-leaf touches both crates.

**Finding 2: existing `Sdf for SignedDistanceField` docstring already warns about the failure mode** (cf-design/src/sdf.rs:102-110). The pre-parry doc said: "for points far from the surface where the closest face's plane happens to lie between the probe and the rest of the body, the heuristic can return the wrong sign. Consumers needing robust far-field inside/outside should prefer `SignedDistanceField::is_inside` (ray-cast based) directly."
→ **Revision**: cited in §7. The pre-parry doc was correct; the parry swap silently made the "prefer is_inside" recommendation also wrong (post-parry `is_inside` is pseudo-normal-based with the same failure mode as `distance`). Argues even more strongly for the architecture fix — the doc-level warning was load-bearing and got silently invalidated.

**Finding 3: more consumers than v1 enumerated.** Grep found:
- `mesh-offset::offset_mesh` (Layer 0 consumer; same MC-grid shape as cf-cast-cli).
- `mesh-lattice::infill` (Layer 0 consumer; explicit inline comment documenting that ray-cast was ~36% wrong).
- Plus the two cf-design internal consumers (`solid_layered.rs` tests, `sdf.rs` itself).
→ **Revision**: §6 now lists 10 migration sites with per-site disposition. D.3 split into D.3 (cf-device-design) + D.4 (mesh-offset + mesh-lattice) + D.5 (cf-cast-cli).

**Finding 4: cf-device-design's flood-fill caches the FULL signed grid, not just sign** (sdf_layers.rs lines 388-530+). The existing code computes unsigned distance per cell, runs the 3-region flood, then writes signed distance into the grid. Queries read from the cached grid via trilinear interpolation; no per-query parry calls.
→ **Revision**: spec now ships TWO primitives — `FloodFillSign` (sign-only cache, for cf-cast-cli's one-shot MC sweep) and `CachedGridSdf` (full distance+sign grid, for cf-device-design's preview cache). Both share the same FloodFill machinery.

**Finding 5: the existing flood-fill is 3-region (Inside / Outside / Wall) with multi-source label expansion**, not 2-region (Inside / Outside). Wall cells are barriers the flood can't cross; the wall threshold is `0.75 × cell_size` (= half-cell + margin, guarantees 6-connectivity watertightness); a second BFS expands Outside/Inside labels into the wall band so every cell carries a sign.
→ **Revision**: §3 + the "headline construction" now reflect the 3-region scheme. `FloodFillReport` (formerly absent in v1) ports the existing `GridSdfReport` diagnostic surface — cell counts per region + inside-component count.

**Finding 6: cf-device-design uses a DECIMATED scan as the SDF source** (`decimate_scan_for_sdf(scan, 2500)` at sdf_layers.rs:394). The decimation is load-bearing for cf-device-design's per-frame budget AND for the heat-map closest-point projection that shares the same mesh. cf-cast-cli doesn't decimate — uses the full 167k-face cleaned scan.
→ **Revision**: §5 (`FloodFillSign::build` signature) now takes any `UnsignedDistance`, so the caller picks decimated vs raw. Banked F2/F3 (drop decimation in cf-device-design / insertion_sim) from the parry-accel spec stays banked — decimation is orthogonal to the sign issue.

---

## Implementation deviations (D.2)

The D.2 implementation followed the spec as-written with two small, documented departures:

**§A — Private `SdfGrid` storage inside mesh-sdf instead of importing the public `ScalarGrid` from mesh-offset.** mesh-offset owns `ScalarGrid` but sits ABOVE mesh-sdf in the L0 tier hierarchy, so importing it in D.2 would have required moving the public type across crates as part of this commit. The local storage is minimal — `Vec<f64>` + dims + origin + cell_size + trilinear sampler + central-diff gradient — and never leaks out of `FloodFillSign` / `CachedGridSdf`. If a downstream consumer ever wants a unified L0 grid type, that's a separate consolidation arc.

**§B — `FloodFillReport` carries `min_signed_distance_m`.** Free correctness contract per [[project-pinned-floor-visual-gate-postmortem]]: `min_signed_distance_m > -bbox_half_diagonal` is a one-line guard against an entire class of sign-flip regressions. Not in spec §3 but cheap to compute during build and useful to surface in build logs.

**Smoothing-free primitives by design.** insertion_sim's existing `build_grid_sdf` runs a separable 3D Gaussian post-pass (σ = 1.0 cell) on the signed buffer for FEM contact-gradient C¹ approximation; sdf_layers does not. Per the stress-test, smoothing is tool-specific and orthogonal to sign correctness. mesh-sdf ships pure flood-fill; insertion_sim re-applies the Gaussian externally during D.3.

## Cold-read entry for next session (D.3)

1. Re-read §6 (per-consumer migration table) + §"Sub-leaf ladder" D.3 in this spec.
2. Read `cf-device-design::sdf_layers::build_cached_scan_sdf` (the canonical 3-region flood-fill) — the migration target. Drop the bespoke `Region` enum + `neighbours6` helper + the 4-pass body in favor of `CachedGridSdf::build(distance, bounds, cell_size, 0.75)`. `closed_grid` retains `ScalarGrid` for the marching-cubes path; this means D.3a unpacks `CachedGridSdf` and re-fills the existing `ScalarGrid` storage rather than rewriting `CachedScanSdf` end-to-end. Keep that scope-tight.
3. Read `cf-device-design::insertion_sim::build_grid_sdf` (the second copy). Migrates same way; keep the Gaussian post-pass external on the result. `GridSdfReport` becomes a thin projection of `FloodFillReport`.
4. Visual gate on iter-1 sock after D.3: cavity + Layer 0 + Layer 1 render unchanged in cf-device-design preview. Pre-migration vs post-migration MC vertices should be bit-equivalent on the sock fixture (modulo the pure-flood-fill vs Gaussian-smoothed signed grid in insertion_sim — that path applies Gaussian externally to preserve bit-equivalence).
5. Run `cargo run -p xtask -- grade cf-device-design` ≥ A.

**Implementation start (D.3)**: `tools/cf-device-design/src/sdf_layers.rs:388-560` (sdf_layers migration) → `tools/cf-device-design/src/insertion_sim.rs:720-820` + `1369-1529` (insertion_sim migration) → `cargo test -p cf-device-design --release` → visual gate.
