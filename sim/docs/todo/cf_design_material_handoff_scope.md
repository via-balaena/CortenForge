# cf-design ↔ sim-soft Material Handoff — Coordination Memo

**Status:** Forward-looking coordination memo. Authored at the close of Phase 4 (`feature/phase-4-multi-material`, commit 13) per scope memo §3 Decision L: "cf-design coordination memo only — no cf-design implementation in Phase 4." This document names the in-memory `(SdfField, MaterialField)` boundary per book Part 7 §00 and lists what `cf-design::Material` must expose to produce a `sim_soft::MaterialField` when the first multi-material consumer lands. **Does not implement any cf-design changes.**

**Authored:** 2026-04-28, Phase 4 close-out.
**Cites:** [`phase_4_multi_material_scope.md`](phase_4_multi_material_scope.md) §3 Decision L; book Part 7 §00 (SDF primitive) + §02 (material assignment); book Part 2 §09 (spatial fields).
**Audience:** the Claude/agent or human picking up cf-design's first multi-material consumer post-Phase-4.

## 0. The boundary in one paragraph

`sim-soft` Phase 4 introduced `MaterialField` as the per-element material aggregator: an in-memory `{ mu: Box<dyn Field<f64>>, lambda: Box<dyn Field<f64>>, interface_sdf: Option<Box<dyn Sdf>> }` that the FEM mesher samples at every tet centroid to produce per-element `NeoHookean` instances. `cf-design` today produces designs in terms of `Material { name, density, youngs_modulus: Option<f64>, color, process }` per part. The handoff is the bridge between cf-design's *design-side* `Material` records (one per labeled region) plus the SDF that partitions space into those regions, and sim-soft's *physics-side* `MaterialField` (per-position scalar Lamé fields). The translation is mechanical — once the design side names its regions and their stiffness pairs, sim-soft's `LayeredScalarField` / `BlendedScalarField` / `MaterialField` constructors compose them — but the cf-design side has to expose enough information for the translation to happen, and that is what this memo specifies.

## 1. What sim-soft `MaterialField` consumes

Phase 4 ships three `Field<f64>` impls and one aggregator:

| Type | Purpose | Construction shape |
|---|---|---|
| `ConstantField<T>` | uniform-everywhere case | `ConstantField::new(value)` |
| `LayeredScalarField` | N-shell concentric step field over `f64` | `LayeredScalarField::new(driving_sdf: Box<dyn Sdf>, thresholds: Vec<f64>, values: Vec<f64>)` — `values.len() == thresholds.len() + 1` |
| `BlendedScalarField` | smoothstep / Hermite sigmoid blend between two `Box<dyn Field<f64>>` weighted by an SDF | `BlendedScalarField::new(sdf, field_a, field_b, blend_half_width)` |
| `MaterialField` | aggregator | `MaterialField::from_fields(mu_field: Box<dyn Field<f64>>, lambda_field: Box<dyn Field<f64>>)` plus optional `.with_interface_sdf(...)` for IV-6 flag population |

The simplest layered-bonded design — the canonical IV-3 / IV-5 layered silicone device, three concentric shells — already maps cleanly: one `SphereSdf` partitioning space, three Lamé pairs, two thresholds. `LayeredScalarField` accepts those four lists directly. The cf-design side needs to surface (a) the SDF, (b) the per-region Lamé pairs, (c) the threshold list along the SDF level set.

Per Decision M (Phase 4 scope memo §3), `MaterialField` is internal-API-shaped — not γ-locked. The design-time API can evolve through Part 10 without breaking sim-soft consumers.

## 2. What cf-design `Material` exposes today

`design/cf-design/src/mechanism/material.rs:90`:

```rust
pub struct Material {
    pub name: String,
    pub density: f64,
    pub youngs_modulus: Option<f64>,
    pub color: Option<[f64; 4]>,
    pub process: Option<ManufacturingProcess>,
}
```

Plus `ManufacturingProcess::{Fdm{..}, Sla{..}, Sls, Machined}`. Today cf-design `Material` records ride on `Part`s of a `Mechanism`; there is no spatial-partition + per-region-material attachment surface. The `mechanism/` module's tendons + parts pin one `Material` per part instance.

## 3. The four gaps to multi-material soft-body consumption

The design-side bridge needs four pieces cf-design today does not expose:

### Gap 1 — `(μ, λ)` pair, not just `youngs_modulus`

Hyperelastic FEM consumes the Lamé pair `(μ, λ)`. cf-design exposes `youngs_modulus: Option<f64>` (Pa) but no `poisson_ratio` field, so the pair is under-determined. **Resolution:** add a `poisson_ratio: Option<f64>` field to `Material`, plus a derived helper `Material::lame_pair() -> Option<(f64, f64)>` that returns `(μ, λ) = (E/(2(1+ν)), Eν/((1+ν)(1−2ν)))` when both are set. For the canonical Decision J / Phase 4 compressible regime the test fixtures pin `ν = 0.4` so `λ = 4μ`; once cf-design owns its own `Material` records, that constant becomes a per-record field.

### Gap 2 — Spatial-partition + per-region-material attachment

A multi-material soft body is `(SdfField, [(RegionLabel, Material)])` — one geometry SDF carving space into labeled regions, one Material per label. cf-design today has no surface for this composition; `Mechanism` carries one `Material` per `Part` but not "this material lives where this SDF level set says." **Resolution:** introduce a new design-side type at the `cf-design` crate level (parallel to `Mechanism`, not inside it):

```rust
// SKETCH ONLY — names + shape are the design question this memo opens,
// not a locked API.
pub struct SoftBody {
    pub body_sdf: Box<dyn Sdf>,         // outer shell + cavity geometry
    pub material_partition: PartitionScheme, // see Gap 3
}
```

`SoftBody` is the cf-design analogue of a multi-material 3D-printed part: a single contiguous body with spatially-varying material. It is NOT a `Mechanism` — `Mechanism` covers articulated assemblies of rigid parts with joints and tendons. Soft bodies live alongside, with their own composition rules and their own translation to sim-soft.

### Gap 3 — Partition scheme: layered vs. blended

Per book Part 7 §02 §00 ("sampling"): material assignment can be a hard step rule (`LayeredScalarField`) or a smooth blend (`BlendedScalarField`). The cf-design surface should name both:

```rust
// SKETCH ONLY.
pub enum PartitionScheme {
    /// N-shell concentric layered partition along an SDF level set.
    /// `(threshold, material)` pairs — material at index 0 wins below
    /// `thresholds[0]`, material at index N wins above `thresholds[N-1]`.
    Layered {
        partitioning_sdf: Box<dyn Sdf>,
        thresholds: Vec<f64>,
        materials: Vec<Material>, // length = thresholds.len() + 1
    },
    /// Smoothstep blend between two material regions across an SDF
    /// transition zone of half-width `L_blend`.
    Blended {
        partitioning_sdf: Box<dyn Sdf>,
        material_a: Material,
        material_b: Material,
        blend_half_width: f64,
    },
    // Phase H additions: `Composite { ... }` for HGO-decorated regions,
    // `Anisotropic { ... }` for fiber-aligned shells, etc.
}
```

The two variants mirror sim-soft's `LayeredScalarField` and `BlendedScalarField` 1:1. The cf-design layer chooses *which* — the sim-soft layer is mechanical translation.

### Gap 4 — Translation: `SoftBody → (Sdf, MaterialField)`

The translation method that produces the in-memory `(SdfField, MaterialField)` tuple sim-soft's `SdfMeshedTetMesh::from_sdf` plus `MeshingHints::material_field` consume:

```rust
// SKETCH ONLY.
impl SoftBody {
    pub fn to_sim_soft(&self) -> Result<(Box<dyn Sdf>, MaterialField), MaterialError> {
        // Geometry: clone body_sdf.
        // MaterialField:
        //   - Layered: build LayeredScalarField for mu + lambda along
        //     partitioning_sdf using each Material's lame_pair().
        //   - Blended: build BlendedScalarField for mu + lambda; inner
        //     fields are ConstantField<f64> from each side's lame_pair.
        // Return (body_sdf, material_field).
    }
}
```

Gap 1's `lame_pair()` helper is the unit of stiffness translation; the rest is sim-soft type-construction at the boundary. The translation is one direction (design → sim) — sim-to-design lifting is a separate concern (the live-remesh / re-sample distinction of book Part 7 §02 §00, deferred to Phase H per Decision O).

## 4. Suggested commit sequencing for the first cf-design consumer

Three commits, foundation-up. Dependency-chain ordered.

1. **Commit 1 — `Material::poisson_ratio` + `Material::lame_pair`.** Add the field + helper. Existing `Material::new` callers untouched (field is `Option<f64>`); `lame_pair()` returns `None` until both `youngs_modulus` and `poisson_ratio` are set. Two unit tests: `Some + Some → Some(μ, λ)` with the Lamé conversion checked at a known reference (e.g., Ecoflex 00-30 at `E = 280 kPa, ν = 0.4` should produce `(μ, λ) = (1e5, 4e5)` at f64 precision after rounding).

2. **Commit 2 — `SoftBody` + `PartitionScheme` + `to_sim_soft` translation.** Three-shell layered Decision-J fixture as the load-bearing test: build a `SoftBody` with three Ecoflex 00-30 / composite / Ecoflex 00-30 records partitioned by a `SphereSdf`, call `to_sim_soft`, assert the returned `MaterialField` produces bit-equal `NeoHookean` instances at known centroids vs. an inline `LayeredScalarField` reference. Mirrors sim-soft's IV-4 region-tagging gate at the cf-design layer.

3. **Commit 3 — End-to-end gate at the design-to-physics boundary.** Take a `SoftBody`, call `to_sim_soft`, feed into `SoftScene::layered_silicone_sphere` (or a Phase-4-follow-on constructor that takes the translated `(Sdf, MaterialField)` directly), run one `replay_step`. Assert cavity-wall radial displacement matches IV-5's analytic Lamé prediction at `cell_size = 0.02` within IV-5's `~0.30` floor. The single regression-net gate that proves the translation does not corrupt physics.

Test scenes deliberately mirror the IV-3 / IV-4 / IV-5 fixtures — when the design layer breaks, the failure mode is shape-isomorphic to the sim-soft side and the diagnosis path is preserved.

## 5. What this memo does NOT cover

- **Sliding interfaces, IPC, or contact.** Phase 5 + Phase H scope. The Phase 4 boundary is bonded multi-material only; sliding contacts at material boundaries belong with the contact-pair surface, not the material-partition surface.
- **Anisotropic / viscoelastic / thermal decoration of cf-design `Material`.** Phase H per scope memo §3 Decision F. Today's `Material` exposes scalar Lamé only; HGO fiber direction, viscoelastic relaxation spectrum, thermal coefficients all need their own decoration surface when their phase arrives.
- **Live re-mesh / re-sample distinction.** Book Part 7 §02 §00 + scope memo §3 Decision O. The translation defined above is the *one-way design-to-physics* direction; the warm-restart / parameter-only-edit distinction is the live-remesh concern, scoped to Phase H Ch 04.
- **Material database / PLA-from-name lookup.** `Material::new("PLA", 1250.0)` already takes name + density inline. A name-keyed canonical-database lookup is a Phase H ergonomics concern, not load-bearing for the bridge defined here.
- **GPU-side `MaterialField`.** Phase E. The current bridge is CPU-only. GPU porting reads the same in-memory `MaterialField` structure and uploads to GPU buffers; no design-side change required.
- **Reverse-mode adjoint w.r.t. material parameters.** Phase 4 scope memo Decision M + post-Phase-H. IV-8 (`tests/material_grad_hook.rs`) is the FD baseline that the eventual reverse-mode adjoint will gradcheck against; the cf-design side does not need to ship the adjoint to use forward sensitivity through its own design loop.

## 6. Open design questions for the consumer commit

These resolve at the first-consumer-commit author's discretion, not preemptively in this memo:

- **Where does `SoftBody` live?** New `cf_design::soft` submodule alongside `mechanism`, OR top-level `cf_design::SoftBody`? Lean: top-level — soft bodies are first-class artifacts, not "soft mechanisms."
- **Should `PartitionScheme::Layered` accept `Vec<Material>` or `Vec<Box<dyn DesignableMaterial>>`?** The latter opens design-time gradient queries via a separate trait surface; the former is mechanically simpler. Lean: `Vec<Material>` for Phase H bridge; introduce a `DesignableMaterial` trait separately when the design-side gradient surface needs it.
- **What error type does `to_sim_soft` return?** A new `MaterialError` enum, OR re-use `sim_soft::MeshingError`, OR `Box<dyn Error>`? Lean: a small new `MaterialError` enum at the cf-design crate level — `MissingPoissonRatio { material_name }`, `LayerCountMismatch { thresholds, materials }`, etc. Keeps cf-design's error surface separate from sim-soft's.
- **Does `SoftBody` carry a default `cell_size` hint?** Mesher tuning is a physics concern, not a design concern; cf-design should not bake mesh resolution into its records. Lean: no — caller of `to_sim_soft` plus `SdfMeshedTetMesh::from_sdf` chooses `cell_size`. Phase H ergonomic-helpers (`SoftBody::recommended_cell_size`) can land later if the per-design heuristic turns out portable.

## 7. Phase H placeholders this memo names

- HGO / viscoelastic / thermal `Material` decoration → Part 2 §06 / §07 / §08 + Phase H.
- High-contrast (mesh-aligned) interfaces → Part 3 §03 §00 + Phase H.
- Sliding / IPC contact at material boundaries → Phase 5 + Phase H.
- Live re-mesh ↔ re-sample distinction in `to_sim_soft` → book Part 7 §02 §00 + Phase H Ch 04.
- Material database (canonical Ecoflex / Dragon Skin / PLA records keyed by name) → Phase H ergonomics.
- Design-time reverse-mode gradient over material parameters → Part 6 §02 IFT-adjoint extension; FD gradcheck reference is `sim/L0/soft/tests/material_grad_hook.rs` (IV-8).

## 8. Why this memo, not a code commit

Phase 4 scope memo §3 Decision L: cf-design coordination is forward-looking only. The first multi-material consumer in cf-design needs the bridge sketched here; building it now would (a) ship machinery without a single consumer driving its API choices, (b) lock decisions before the cf-design loop has been exercised end-to-end against sim-soft Phase 4 plumbing, and (c) introduce cf-design changes inside a sim-soft-only PR. The cleaner sequencing is: Phase 4 ships the in-memory `MaterialField` structure cf-design will consume, this memo names the boundary, the next cf-design design loop opens its own PR with the bridge — written by an author who can let the consumer drive the API.

The memo ends with the bridge unblocked, not built. That is the deliverable.
