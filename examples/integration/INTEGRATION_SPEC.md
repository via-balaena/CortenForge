# Integration Examples Spec

> Four examples that prove the CortenForge value proposition:
> **the design geometry IS the simulation collider IS the 3D-printable artifact.**

---

## Motivation

CortenForge has three domains (design, mesh, sim) that each work in isolation.
The existing `design-to-sim` example demonstrates one seam — design-to-physics
— but uses manual mesh syncing (`to_stl_kit` + per-body spawning) instead of
the engine's own `spawn_model_geoms`. Two critical integration paths are
undemonstrated:

1. **Design → Print** — lattice infill today fills a bounding box, not the
   designed shape.
2. **Sim → Design feedback** — simulation force data should drive
   variable-density lattice generation.

These four examples close those gaps with minimal library changes (~60 lines)
and serve as ongoing acceptance tests for cross-domain integration.

---

## The Four Examples

### 1. `design-to-sim` (fix existing)

**Proves:** Design geometry is directly usable as a simulation collider with
zero export steps.

**Current state:** The example manually calls `mechanism.to_stl_kit(0.3)`,
spawns each part mesh by hand, and writes a custom `sync_bodies` system that
reads `data.xpos`/`data.xquat`. This duplicates what `spawn_model_geoms` +
`sync_geom_transforms` already do — except `spawn_model_geoms` skips
`GeomType::Mesh` geoms (line 520 of `model_data.rs`: `_ => None`).

**Library fix (~15 lines in `sim/L1/bevy/src/model_data.rs`):**

In `spawn_model_geoms`, the existing match (lines 504–522) builds an
`Option<Shape>` then calls `mesh_from_shape()`. Since `GeomType::Mesh` produces
a `Mesh` directly (bypassing the `Shape` intermediary), the code needs minor
restructuring. Refactor the mesh-production logic so that the `GeomType::Mesh`
case merges into the same `Option<Mesh>` binding:

```rust
// Produce a Bevy mesh for this geom
let mesh: Option<Mesh> = match geom_type {
    GeomType::Mesh => {
        // Look up mesh asset: geom_mesh[geom_id] → mesh_data[mesh_id]
        model.geom_mesh[geom_id].and_then(|mesh_id| {
            let tri_data = &model.mesh_data[mesh_id];
            let indexed = Arc::new(tri_data.indexed_mesh().clone());
            Some(triangle_mesh_from_indexed(&indexed))
        })
    }
    _ => {
        // Primitive shapes go through Shape → mesh_from_shape
        let shape = match geom_type {
            GeomType::Plane => Some(Shape::Plane { ... }),
            // ... existing arms ...
            _ => None,
        };
        shape.and_then(|s| mesh_from_shape(&s))
    }
};

let Some(mesh) = mesh else { continue };
```

The world-frame transform from `data.geom_xpos`/`data.geom_xmat` already
applies to all geom types — no changes needed downstream.

**Imports needed:** `std::sync::Arc` and `crate::mesh::triangle_mesh_from_indexed`
(neither is currently imported in `model_data.rs`).

**Example rewrite:** Remove manual `to_stl_kit` + custom `sync_bodies`. Use
`spawn_model_geoms` for spawning + `sync_geom_transforms` for animation.
The example becomes ~100 lines shorter and proves the zero-export-step claim.

```rust
// setup() — simplified
let mechanism = build_gripper();
let mjcf_xml = mechanism.to_mjcf(1.5);
let model = sim_mjcf::load_model(&mjcf_xml).expect("MJCF should load");
let mut data = model.make_data();
let _ = data.forward(&model);

// Spawn ALL geoms (including mesh geoms) via the engine
spawn_model_geoms(&mut commands, &mut meshes, &mut materials, &model, &data);

// Physics resources
commands.insert_resource(PhysicsModel(model));
commands.insert_resource(PhysicsData(data));
```

Systems: `step_model_data` + `sync_geom_transforms` (both from sim-bevy) +
`actuate_gripper` (example-local).

**Acceptance criteria:**
- Gripper animates with design meshes rendered by `spawn_model_geoms`
- No `to_stl_kit` call in example code
- No custom body-sync system — uses `sync_geom_transforms`
- `cargo test -p sim-bevy` passes (new test for mesh geom spawning)

---

### 2. `design-to-print` (new)

**Proves:** Lattice infill conforms to the designed shape boundary, not a
bounding box.

**The problem:** `generate_lattice(params, bounds)` fills the entire AABB.
When you lattice-infill a curved part (e.g., a rounded bracket), struts and
TPMS surfaces extend outside the designed boundary. The result isn't printable
without manual trimming.

**Library fix (~35 lines across 2 files in `mesh/mesh-lattice/`):**

#### `src/params.rs` — add `shape_sdf` field + builder method (~15 lines)

```rust
// New field in LatticeParams:
/// Optional shape SDF for boundary-conforming lattice generation.
///
/// When set, lattice geometry outside the shape boundary (where
/// `sdf(point) > 0`) is excluded. The SDF convention is:
/// - Negative = inside the shape
/// - Positive = outside the shape
/// - Zero = on the surface
pub shape_sdf: Option<Arc<dyn Fn(Point3<f64>) -> f64 + Send + Sync>>,

// New builder method:
/// Sets a shape SDF for boundary-conforming lattice generation.
///
/// Only lattice geometry inside the shape (where `sdf(point) <= 0`)
/// is generated. This allows lattice infill to conform to arbitrary
/// shapes rather than filling an axis-aligned bounding box.
///
/// # Examples
///
/// ```
/// use mesh_lattice::LatticeParams;
/// use std::sync::Arc;
///
/// // Sphere boundary: only generate lattice inside radius 25
/// let params = LatticeParams::gyroid(8.0)
///     .with_shape_sdf(Arc::new(|p| {
///         (p.x * p.x + p.y * p.y + p.z * p.z).sqrt() - 25.0
///     }));
/// ```
#[must_use]
pub fn with_shape_sdf(
    mut self,
    sdf: Arc<dyn Fn(Point3<f64>) -> f64 + Send + Sync>,
) -> Self {
    self.shape_sdf = Some(sdf);
    self
}

/// Returns true if the point is outside the shape boundary.
///
/// When no shape SDF is set, all points are considered inside.
#[must_use]
pub fn is_outside_shape(&self, point: Point3<f64>) -> bool {
    self.shape_sdf
        .as_ref()
        .map_or(false, |sdf| sdf(point) > 0.0)
}
```

Default: `shape_sdf: None` (existing behavior preserved).

**Note:** `LatticeParams` currently derives `Debug`. Since `dyn Fn` does not
implement `Debug`, the derive must be replaced with a manual `impl Debug` that
prints `"<shape_sdf>"` for the closure field. This matches the existing
precedent in `DensityMap` (`density.rs` lines 91–145) which faces the same
issue with its `StressField` and `Function` variants.

#### `src/generate.rs` — boundary checks in generators (~20 lines)

**Strut-based generators** (cubic, octet-truss, voronoi): Before generating
struts from a node, check `params.is_outside_shape(node)`. Skip if outside.

```rust
// In generate_cubic_lattice, after density check (line ~137):
if params.is_outside_shape(node) {
    continue;
}

// Same pattern in generate_octet_truss_lattice (check center point)
// and generate_voronoi_lattice (check node point).
```

**TPMS generators** (gyroid, schwarz-p, diamond): Intersect the shape SDF
with the TPMS shell SDF via `max()`:

```rust
// In generate_tpms_lattice, replace the shell_sdf closure:
let shell_sdf = move |p: Point3<f64>| -> f64 {
    let tpms_value = (tpms_fn(p) - threshold).abs() - half_thickness;
    match &shape_sdf_clone {
        Some(sdf) => tpms_value.max(sdf(p)),  // intersect with shape
        None => tpms_value,
    }
};
```

This ensures TPMS lattice surfaces are trimmed to the shape boundary.

**Example pipeline:**

```
Design Solid → mesh() → repair → shell → lattice (with_shape_sdf) → printability → Bevy
```

```rust
// Core pipeline
let solid = Solid::cuboid(Vector3::new(30.0, 20.0, 15.0))
    .round(3.0)
    .subtract(Solid::cylinder(4.0, 20.0).translate(Vector3::new(10.0, 0.0, 0.0)));

let mut mesh = solid.mesh(0.5);
mesh_repair::repair_mesh(&mut mesh, &mesh_repair::RepairParams::default());

let shell = mesh_shell::ShellBuilder::new(&mesh)
    .wall_thickness(1.2)
    .fast()
    .build()
    .expect("shell generation");

// Shape-conforming lattice
let sdf = {
    let s = solid.clone();
    Arc::new(move |p: Point3<f64>| s.evaluate(&p))
};
let lattice_params = LatticeParams::gyroid(6.0)
    .with_density(0.25)
    .with_shape_sdf(sdf);
let dims = mesh_measure::dimensions(&mesh);
let bounds = (dims.min, dims.max);
let lattice = generate_lattice(&lattice_params, bounds)?;

// Printability check
let report = mesh_printability::validate_for_printing(
    &lattice.mesh, &mesh_printability::PrinterConfig::fdm_default(),
)?;
```

**Bevy visualization:** 3 stages side-by-side (original design, shell, lattice
infill), color-coded, orbit camera.

**Acceptance criteria:**
- Lattice mesh has zero vertices outside the shape boundary (within tolerance)
- Printability report passes (manifold, min wall thickness met)
- `cargo test -p mesh-lattice` passes (new tests for shape_sdf)
- Visual: lattice clearly conforms to the rounded bracket silhouette

---

### 3. `sim-informed-design` (new)

**Proves:** Simulation force data drives variable-density lattice. This is the
feedback loop: simulate → analyze → redesign.

**No library changes.** All bridge logic lives in the example.

**Pipeline:**

```
Design bracket → simulate with load → extract contact forces →
build stress field → stress-graded lattice inside shape → visualize
```

**Stress field construction (example code):**

Contact forces are not stored per-contact in sim-core. The Data struct stores
contacts as `data.contacts: Vec<Contact>` (each with `.pos`, `.normal`,
`.depth`, `.geom1`, `.geom2`) and constraint forces in the unified
`data.efc_force: DVector<f64>` indexed by constraint row. For a demonstration
stress field, we use body-level `cfrc_ext` (accumulated contact + external
forces per body) which is simpler and sufficient:

```rust
/// Build a gaussian-weighted stress field from body-level force data.
///
/// Uses `data.cfrc_ext` (spatial force on each body from contacts + external)
/// and contact positions as stress sources. This is a simplified stress
/// proxy — not FEA — but sufficient for density-grading lattice infill.
fn build_stress_field(
    data: &Data,
    model: &Model,
    sigma: f64,
) -> Arc<dyn Fn(Point3<f64>) -> f64 + Send + Sync> {
    let mut stress_sources: Vec<(Point3<f64>, f64)> = Vec::new();

    // Body-level forces from cfrc_ext [torque(3), force(3)]
    for body_id in 1..model.nbody {
        let force = &data.cfrc_ext[body_id];
        let linear_mag = Vector3::new(force[3], force[4], force[5]).norm();
        if linear_mag > 1e-6 {
            let pos = &data.xpos[body_id];
            stress_sources.push((Point3::new(pos.x, pos.y, pos.z), linear_mag));
        }
    }

    // Contact positions (with penetration depth as magnitude proxy)
    for contact in &data.contacts {
        if contact.depth > 1e-8 {
            let pos = contact.pos;
            stress_sources.push((
                Point3::new(pos.x, pos.y, pos.z),
                contact.depth * 1000.0, // scale depth to force-like magnitude
            ));
        }
    }

    let inv_2sigma2 = 1.0 / (2.0 * sigma * sigma);

    Arc::new(move |p: Point3<f64>| -> f64 {
        stress_sources.iter().map(|(src, mag)| {
            let dist2 = (p - src).norm_squared();
            mag * (-dist2 * inv_2sigma2).exp()
        }).sum()
    })
}
```

**Note on `cfrc_ext`:** This field is populated by `mj_body_accumulators()`
during `forward()` / constraint solving. Ensure the simulation has run enough
steps for meaningful contact data before extracting the stress field.

**Density map from stress field:**

```rust
let stress_fn = build_stress_field(&data, &model, 8.0);
let density_map = DensityMap::from_stress_field(stress_fn, 0.15, 0.6);
```

**Lattice generation with both stress density and shape conformance:**

```rust
let lattice_params = LatticeParams::gyroid(5.0)
    .with_density_map(density_map)
    .with_shape_sdf(shape_sdf);
let lattice = generate_lattice(&lattice_params, bounds)?;
```

**Bevy visualization:** Side-by-side: (1) bracket under load with contact
forces as colored spheres, (2) stress-graded lattice showing visible density
gradient (denser near contacts, sparser away).

**Acceptance criteria:**
- Lattice density is measurably higher near contact/force regions
- Lattice conforms to shape boundary (uses `with_shape_sdf` from Phase 2)
- Console output shows stress field statistics (min/max/mean stress, density
  range)
- Visual: clear density gradient visible in the lattice

---

### 4. `full-pipeline` (capstone)

**Proves:** All three domains compose end-to-end. This is Milestone 2 from
`docs/VISION.md`.

**No new library changes.** Composes patterns from examples 1–3.

**Pipeline:**

```
Design mechanism → simulate → extract stress → per-part lattice →
shell → validate printability → 5-stage Bevy visualization
```

**5 visualization stages** (user presses Space to advance):

| Stage | Shows | Purpose |
|-------|-------|---------|
| 1. Design | Solid mechanism (palm + 2 fingers) | "Here's what we designed" |
| 2. Simulate | Animated gripper closing on object | "Here's how it behaves" |
| 3. Stress | Colored overlay showing force distribution | "Here's where the stress is" |
| 4. Lattice | Per-part stress-graded lattice infill | "Here's the optimized infill" |
| 5. Print-ready | Shelled + lattice parts with printability badge | "Here's what we manufacture" |

**Per-part processing:**

```rust
let stl_kit = mechanism.to_stl_kit(0.3);

for (name, mut part_mesh) in stl_kit {
    mesh_repair::repair_mesh(&mut part_mesh, &mesh_repair::RepairParams::default());

    let shell = mesh_shell::ShellBuilder::new(&part_mesh)
        .wall_thickness(1.0)
        .fast()
        .build()
        .expect("shell");

    // Per-part stress field from simulation
    let part_stress = extract_part_stress(&data, &model, &name);
    let density_map = DensityMap::from_stress_field(part_stress, 0.15, 0.55);

    // Shape-conforming lattice — look up the part's Solid
    let solid = mechanism.parts().iter()
        .find(|p| p.name() == name)
        .expect("part exists")
        .solid()
        .clone();
    let sdf = Arc::new(move |p: Point3<f64>| solid.evaluate(&p));

    let dims = mesh_measure::dimensions(&part_mesh);
    let params = LatticeParams::gyroid(4.0)
        .with_density_map(density_map)
        .with_shape_sdf(sdf);
    let lattice = generate_lattice(&params, (dims.min, dims.max))?;

    // Printability
    let report = mesh_printability::validate_for_printing(
        &lattice.mesh, &mesh_printability::PrinterConfig::fdm_default(),
    )?;
    println!("  {name}: printable={} — {}", report.is_printable(), report.summary());
}
```

**Acceptance criteria:**
- All 5 stages render correctly
- Space bar advances stages
- Each part's printability report is printed to console
- All lattice parts conform to their shape boundaries
- Density gradient is visible (denser at contact regions)
- `cargo clippy -- -D warnings` passes on all modified crates

---

## Sequencing

```
Phase 1: design-to-sim ──────────┐
                                  ├── Phase 3: sim-informed-design ──┐
Phase 2: design-to-print ────────┘                                   ├── Phase 4: full-pipeline
                                                                      │
                                  (Phase 2 shape_sdf also needed) ───┘
```

- **Phases 1 & 2** are independent — can be implemented in parallel.
- **Phase 3** needs both: mesh geom rendering from Phase 1 for the sim
  visualization, `shape_sdf` from Phase 2 for boundary-conforming lattice.
- **Phase 4** needs all three patterns.

---

## Library Changes Summary

| File | Change | Lines | Tests |
|------|--------|-------|-------|
| `sim/L1/bevy/src/model_data.rs` | `GeomType::Mesh` arm + refactored mesh dispatch | ~15 | spawn mesh geom test |
| `mesh/mesh-lattice/src/params.rs` | `shape_sdf` field + `with_shape_sdf()` + `is_outside_shape()` + manual `Debug` impl | ~30 | builder + boundary tests |
| `mesh/mesh-lattice/src/generate.rs` | SDF boundary checks in 4 generators | ~15 | shape-conforming lattice tests |

**Total library code: ~60 lines across 3 files.** Everything else is example code.

No new crate dependencies. No changes to Layer 0 sim crates. The mesh-lattice
changes are Layer 0 (Bevy-free). The sim-bevy change is Layer 1.

---

## Quality Criteria (per STANDARDS.md)

- [ ] Unit tests for each library change
- [ ] Doc comments on all new public APIs (with `# Examples`)
- [ ] Zero clippy warnings (`cargo clippy -- -D warnings`)
- [ ] No `unwrap()`/`expect()` in library code
- [ ] No new crate dependencies
- [ ] Bevy-free for L0 changes (`mesh-lattice`)
- [ ] Builder pattern consistent with existing API (`with_*()` methods)
- [ ] Each example runs with `cargo run -p <name> --release`

---

## Verification

After implementation:

```bash
# Library tests
cargo test -p sim-bevy          # mesh geom rendering
cargo test -p mesh-lattice      # shape_sdf constraint

# Examples
cargo run -p example-design-to-sim --release           # gripper with engine-rendered meshes
cargo run -p example-design-to-print --release         # lattice conforms to shape
cargo run -p example-sim-informed-design --release     # visible density gradient
cargo run -p example-full-pipeline --release           # 5-stage visualization

# Quality
cargo clippy -p sim-bevy -p mesh-lattice -- -D warnings
```

---

## Session Plan

### Session 1 — `design-to-sim` library fix + example rewrite

**Scope:**
- Add `GeomType::Mesh` handling in `spawn_model_geoms` (~15 lines)
- Add imports (`Arc`, `triangle_mesh_from_indexed`) to `model_data.rs`
- Write unit test for mesh geom spawning in sim-bevy
- Rewrite `examples/integration/design-to-sim/` to use `spawn_model_geoms` +
  `sync_geom_transforms` (net deletion — example gets ~100 lines shorter)

**Entry:** Branch is clean, existing design-to-sim example runs.
**Exit:** `cargo test -p sim-bevy` passes, `cargo run -p example-design-to-sim --release`
shows animated gripper with engine-rendered meshes, no `to_stl_kit` in example code.

---

### Session 2a — `shape_sdf` library change in mesh-lattice

**Scope:**
- Add `shape_sdf` field to `LatticeParams` in `params.rs`
- Replace `#[derive(Debug)]` with manual `Debug` impl
- Add `with_shape_sdf()` builder + `is_outside_shape()` helper
- Add boundary checks in all 4 generators in `generate.rs`
  (cubic, octet-truss, voronoi: skip outside nodes; TPMS: `max()` intersection)
- Write unit tests: builder round-trip, sphere boundary conformance, TPMS
  trimming, backward compat (no sdf = same as before)

**Entry:** Session 1 committed (or independent — no dependency).
**Exit:** `cargo test -p mesh-lattice` passes, `cargo clippy -p mesh-lattice -- -D warnings` clean.

---

### Session 2b — `design-to-print` example

**Scope:**
- Create `examples/integration/design-to-print/` (Cargo.toml + main.rs)
- Register in workspace `Cargo.toml`
- Pipeline: design solid → mesh → repair → shell → lattice with `shape_sdf` →
  printability → 3-stage Bevy visualization

**Entry:** Session 2a committed (needs `shape_sdf` API).
**Exit:** `cargo run -p example-design-to-print --release` shows lattice
conforming to shape boundary. Console prints printability report.

---

### Session 3 — `sim-informed-design` example

**Scope:**
- Create `examples/integration/sim-informed-design/` (Cargo.toml + main.rs)
- Register in workspace `Cargo.toml`
- Implement `build_stress_field()` using `cfrc_ext` + contact positions
- Pipeline: design bracket → simulate N steps → extract stress → density map →
  stress-graded lattice with `shape_sdf` → 2-panel Bevy visualization

**Entry:** Sessions 1 + 2a committed (needs mesh geom rendering + `shape_sdf`).
**Exit:** `cargo run -p example-sim-informed-design --release` shows visible
density gradient. Console prints stress field statistics.

---

### Session 4 — `full-pipeline` capstone example

**Scope:**
- Create `examples/integration/full-pipeline/` (Cargo.toml + main.rs)
- Register in workspace `Cargo.toml`
- Compose patterns from Sessions 1–3: mechanism → simulate → stress → per-part
  lattice → shell → printability
- 5-stage Bevy visualization with Space-bar state machine
- Final `cargo clippy` pass on all modified crates

**Entry:** Sessions 1–3 committed.
**Exit:** `cargo run -p example-full-pipeline --release` renders all 5 stages,
Space advances stages, console prints per-part printability. All quality
criteria met.

---

### Session dependency graph

```
Session 1 (design-to-sim) ───────────┐
                                      ├── Session 3 (sim-informed-design) ──┐
Session 2a (shape_sdf lib) ──┐       │                                      │
                              ├───────┘                                      ├── Session 4 (capstone)
Session 2b (design-to-print) ┘                                              │
                                                                             │
                              (2b needs 2a; 3 needs 1+2a; 4 needs all) ─────┘
```

Sessions 1 and 2a are independent — can run in parallel.
Session 2b depends only on 2a.
Session 3 depends on 1 + 2a.
Session 4 depends on all.

---

## Non-Goals

- **No new crate dependencies.** The entire spec is achievable with existing
  APIs.
- **No physics changes.** sim-core is untouched. The mesh geom fix is
  Layer 1 only (rendering, not collision).
- **No mesh pipeline changes** beyond the `shape_sdf` addition to
  mesh-lattice. `mesh-repair`, `mesh-shell`, `mesh-printability` are used
  as-is.
- **No performance optimization.** These are demonstration examples, not
  production pipelines. Correctness first.

---

*Last updated: 2026-03-20*
