# Output formats and process planning

What this part covers (when authored to depth):

The casting domain emits two kinds of output: geometry files for fabricating the molds, and process documents for executing the cast.

## Mold geometry files

The mold is what gets fabricated (3D-printed, CNC'd, etc.). It needs to leave the casting toolchain in a format the fab equipment consumes.

- **STL.** The lingua franca of 3D printing. Triangulated boundary mesh, no units (interpreted as millimeters by convention), no material info. Adequate for printed PLA / SLA molds.
- **STEP / IGES.** Parametric solid model for CNC and traditional CAM workflows. Carries units, hierarchy, and feature semantics. Required for CNC'd metal molds and any case where the fab side wants to add tooling features.
- **3MF.** A more modern STL replacement; carries materials, colors, multi-part assembly. Worth supporting for multi-piece molds where the assembly relationship matters.
- **SDF directly.** For fab equipment that consumes SDFs natively (some research-grade CNC pipelines, some volumetric 3D printers). Niche today, possibly mainstream in five years.

The SDF → mesh path internally uses the same `sim-soft::sdf_bridge` infrastructure the simulator uses. Mesh quality requirements differ — fab tolerances are coarser than FEM tolerances, and the export mesh can be much lighter than the simulation mesh.

## Casting sequence documentation

The "instructions" for fabricating the part. For a multi-shot cast this is a real document:

- Per-shot ordering with material, mix ratio, working time.
- Pour order within each shot (port → cavity → vents).
- Demold timing per shot (cure clock).
- Adhesion strategy at each inter-shot seam.
- Placement transforms between shots.
- Cure environment per shot (ambient temperature, oven post-cure, humidity).

Output formats:

- **Markdown / HTML.** Human-readable. Default for one-off bench casts.
- **JSON / YAML.** Machine-readable. Required if a fab robot or automation pipeline executes the cast.
- **PDF.** Print-and-pin-to-the-bench format. Worth supporting because that's how human casters actually work.

## Bill of materials

What the cast actually consumes:

- Total mass / volume per material.
- Total mold material per fab pass (PLA filament, aluminum stock).
- Consumables (mixing cups, gloves, mold-release spray, primer).
- Tooling (vacuum chamber time, oven time).

The BOM enables cost estimation per design — feeds into the manufacturability reward term per [Part 6](60-loop-integration.md).

## Inspection / measurement plan

The outer-loop measurement step (per [soft-body full loop](../../soft_body_architecture/src/100-optimization/06-full-loop.md)) needs to know what to measure. Casting-specific measurements:

- Dimensional accuracy at named features (compare measured radii / wall thicknesses to design).
- Bubble count and location (visual inspection or CT).
- Surface roughness at named regions.
- Per-shot bond quality (peel test, ultrasound for non-destructive cases).

Emitting an inspection plan alongside the casting plan closes the loop: every design that gets fabricated has a measurement protocol that produces data the sim-to-real residual GP can ingest.

## Programmatic surface

```rust
pub struct CastOutputBundle {
    pub mold_files: Vec<MoldFile>,           // STL / STEP / 3MF per piece
    pub sequence_doc: SequenceDocument,       // Markdown + JSON
    pub bill_of_materials: BillOfMaterials,
    pub inspection_plan: InspectionPlan,
}

impl CastSequence {
    pub fn emit(&self, output_dir: &Path, formats: &OutputFormats)
        -> Result<CastOutputBundle, EmitError>;
}
```

The `OutputFormats` argument is opt-in — bench users want STL + Markdown, automated fab pipelines want STEP + JSON, paper-publication renders want all four formats.

## Open questions

- How to version-control the bundle. Multi-shot casts produce 10+ files per design; storing them per-design under git LFS is awkward. Probably needs a lightweight artifact store.
- How to track which physical cast instance came from which design version. Provenance metadata embedded in the bundle, ideally.
