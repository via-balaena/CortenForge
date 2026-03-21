# Spec: SDF-Native Physics — Make the Implicit Surface the Physical Object

## Context

The finger-design example currently has a split personality:
- **Design mesh** (left, static): high-res mesh from `Solid::mesh(0.3)` — shows socket, condyle, pin bore
- **Physics mesh** (right, animated): coarse mesh from `to_mjcf(0.5)` — different resolution, mesh artifacts ("crumbs") at thin features like the 0.6mm socket wall

The goal: **SDF === physics collision**. The implicit surface (`Solid`) should
BE the physical object. No lossy mesh intermediary for collision. What you see is
what physics simulates.

This also enables physically accurate articulation — the condyle rotates within
the socket void, constrained by the socket wall geometry, exactly like a real
knuckle joint.

## What Already Exists

CortenForge's sim-core is a **pure Rust physics engine** (no MuJoCo C bindings).
The infrastructure for SDF-based physics is already built:

| Component | File | Status |
|-----------|------|--------|
| `GeomType::Sdf` | `sim/L0/core/src/types/enums.rs` | Exists |
| `Model.sdf_data: Vec<Arc<SdfGrid>>` | `sim/L0/core/src/types/model.rs:557` | Exists |
| `Model.geom_sdf: Vec<Option<usize>>` | `sim/L0/core/src/types/model.rs:330` | Exists |
| `SdfGrid` (distance + gradient queries) | `design/cf-geometry/src/sdf.rs` | Exists |
| `SdfGrid::from_fn()` (build grid from eval function) | `design/cf-geometry/src/sdf.rs:123` | Exists |
| `collide_with_sdf()` (full dispatch: 10 handlers + SDF↔SDF) | `sim/L0/core/src/collision/sdf_collide.rs` | Exists |
| `Solid::evaluate()` (exact SDF value at any point) | `design/cf-design/src/evaluate.rs` | Exists |
| `Solid::gradient()` (analytic gradient via chain rule) | `design/cf-design/src/gradient.rs` | Exists |
| `Solid::bounds()` (AABB for grid sizing) | `design/cf-design/src/solid.rs:1065` | Exists |
| `DISABLE_FILTERPARENT` (enable parent-child collision) | `sim/L0/core/src/collision/mod.rs:91` | Exists |
| Plugin system with `PluginCapabilityBit::Sdf` | `sim/L0/core/src/plugin.rs` | Exists |
| `TriangleMeshData` (visual mesh storage in Model) | `sim/L0/core/src/mesh.rs` | Exists |
| `spawn_model_geoms` (Bevy mesh rendering from Model) | `sim/L1/bevy/src/model_data.rs:493` | Exists |
| `sync_geom_transforms` (apply physics poses to visuals) | `sim/L1/bevy/src/model_data.rs:300` | Exists |

**The gap is narrow:** connect `Solid` → `SdfGrid` → `Model` and bypass the
MJCF XML round-trip. The collision pipeline is already fully wired.

## Architecture

```
Solid (exact implicit surface)
  │
  ├──→ SdfGrid (discretized for physics collision)
  │       cell_size ≤ thinnest wall (e.g. 0.5mm for 0.6mm socket wall)
  │       stored in Model.sdf_data[sdf_id]
  │       referenced by GeomType::Sdf geom via geom_sdf[geom_id]
  │       collision dispatch: collide_with_sdf() → sdf_*_contact()
  │
  └──→ mesh(0.3) (high-res triangle mesh for visual display)
          stored in Model.mesh_data[mesh_id]
          referenced by GeomType::Mesh geom (visual-only, non-colliding group)
          rendered in Bevy via spawn_model_geoms → sync_geom_transforms
```

**Key invariant:** Both representations derive from the identical `Solid` — same
mathematical object, two views optimized for their respective jobs. SDF for O(1)
collision queries with exact distance semantics. Mesh for GPU rendering.

### Parent-Child SDF Collision

For the finger: the condyle (proximal SDF) and socket (distal SDF) are physical
features. With `DISABLE_FILTERPARENT` set, SDF collision between parent and child
bodies is active. The SDF naturally encodes:

- **Socket wall** (SDF < 0): solid material — generates repulsive contact force
- **Socket void** (SDF > 0): empty space where condyle lives — no contact
- **Socket surface** (SDF = 0): contact boundary — contact normal from `gradient()`

When the condyle moves within the void, no contact. When it pushes against the
wall, the SDF collision generates the correct contact point, normal, and
penetration depth. Exactly how a real bearing/knuckle works.

## Changes

### 1. `Mechanism::to_model()` — Direct Model construction

**File: `design/cf-design/src/mechanism/model_builder.rs` (NEW)**

Build a sim-core `Model` directly from a `Mechanism`, bypassing the MJCF XML
round-trip. This eliminates the mesh-in-XML bottleneck and enables SDF geoms.

```rust
impl Mechanism {
    /// Build a physics Model directly from the mechanism.
    ///
    /// - `sdf_resolution`: SdfGrid cell size (mm). Must be ≤ thinnest wall.
    /// - `visual_resolution`: triangle mesh resolution for Bevy rendering.
    pub fn to_model(&self, sdf_resolution: f64, visual_resolution: f64) -> sim_core::Model;
}
```

For each Part:
1. `solid.bounds()` → AABB → pad by `sdf_resolution * 2`
2. `SdfGrid::from_fn(w, h, d, sdf_resolution, origin, |p| solid.evaluate(&p))`
3. `solid.mesh(visual_resolution)` → `TriangleMeshData` (for Bevy)
4. Two geoms per part:
   - **Collision geom**: `GeomType::Sdf`, `geom_sdf[id] = Some(sdf_id)`, `contype=1, conaffinity=1`
   - **Visual geom**: `GeomType::Mesh`, `geom_mesh[id] = Some(mesh_id)`, `contype=0, conaffinity=0` (no collision)

Body hierarchy, joints, tendons, actuators: reuse the same logic from `mjcf.rs`
(joint anchors, body positions, geom offsets via `compute_geom_offset`).

Set `DISABLE_FILTERPARENT` on the model so parent-child SDF collision is active.

### 2. `spawn_model_geoms` — Already handles this

No changes needed. `GeomType::Mesh` geoms (visual) render normally.
`GeomType::Sdf` geoms (collision) hit the `_ => None` branch and are skipped
(invisible) — which is correct since the visual mesh geom handles display.

### 3. Update finger example

**File: `examples/fundamentals/finger-design/src/main.rs`**

Replace:
```rust
let mjcf_xml = mechanism.to_mjcf(0.5);
let model = sim_mjcf::load_model(&mjcf_xml).expect("...");
```

With:
```rust
let model = mechanism.to_model(0.5, 0.3);
// sdf_resolution=0.5mm: resolves 0.6mm socket wall (~1.2 cells)
// visual_resolution=0.3mm: high-res display mesh (matches static view)
```

## Files

| File | Change |
|------|--------|
| `design/cf-design/src/mechanism/model_builder.rs` | NEW — `Mechanism::to_model()` |
| `design/cf-design/src/mechanism/mod.rs` | Add `mod model_builder`, expose `to_model` on Mechanism |
| `design/cf-design/Cargo.toml` | Add `sim-core`, `cf-geometry` dependencies |
| `examples/fundamentals/finger-design/src/main.rs` | Use `to_model()` instead of `to_mjcf()` |

## Open Questions

1. **SDF grid resolution vs memory/speed.** A 40mm finger at 0.5mm cell size ≈
   80×60×80 = 384K cells × 8 bytes ≈ 3MB per part. At 0.3mm ≈ 133×100×133 ≈
   1.77M cells ≈ 14MB. Acceptable?

2. **SDF-vs-SDF collision quality.** `sdf_sdf_contact()` exists but may need
   stress-testing with real socket/condyle geometry. Multi-contact? Stability?

3. **MJCF coexistence.** Should `to_model()` fully replace `to_mjcf()`, or
   live alongside it? (Proposal: alongside — MJCF is still useful for export
   to external tools.)

4. **Plugin vs SdfGrid.** The Plugin system (`PluginCapabilityBit::Sdf`) could
   wrap `Solid::evaluate()` directly (no grid discretization — exact evaluation
   per query). Higher ceiling but slower per-query (tree traversal vs trilinear
   interpolation). Worth benchmarking.

## Verification

```bash
cargo test -p cf-design                          # existing tests pass
cargo test -p cf-design -- model_builder         # new model builder tests
cargo clippy -p cf-design -- -D warnings
cargo clippy -p example-finger-design -- -D warnings
cargo run -p example-finger-design --release     # visual matches physics
```

Expected:
- Socket and condyle are physical — condyle rotates within the socket void
- Visual mesh matches the SDF collision boundary (same Solid)
- No crumbs — visual is high-res mesh, collision is SDF grid
- Parent-child collision active — socket walls resist lateral forces
