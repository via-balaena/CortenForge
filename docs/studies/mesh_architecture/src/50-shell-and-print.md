# mesh-shell and mesh-printability — manufacturing-aware operations

What this part covers (when authored to depth):

This part covers two crates that share a thesis: **manufacturing constraints belong in the geometry layer, not bolted on as an afterthought**. `mesh-shell` produces printable shells (offset-based geometry that respects wall-thickness constraints); `mesh-printability` validates printability against printer-specific configurations. Both inform — and are foundational to — the casting domain's manufacturability story (see [Casting book Part 5](../../casting_architecture/src/50-constraints.md)).

## `mesh-shell` — printable shells

```rust
let shell = ShellBuilder::new(&mesh)
    .wall_thickness(2.0)        // 2mm walls
    .voxel_size(0.5)            // SDF grid resolution
    .high_quality()             // SDF-based vs normal-based
    .build()?;
```

A `ShellBuilder` consumes a closed triangle mesh and produces a watertight shell — inner surface (offset inward by `wall_thickness`), outer surface (the original), and a rim closing the open boundary. Output is a single watertight `IndexedMesh` ready for printing.

Two wall-generation methods:

| Method | Speed | Wall thickness uniformity |
|---|---|---|
| **Normal-based** (`.fast()`) | fast | varies at corners (each vertex offset along its normal; corners get geometric distortion) |
| **SDF-based** (`.high_quality()`) | slower | uniform everywhere (offset inward via `mesh-offset`'s SDF + marching-cubes path) |

Optional pre-pass `ShellBuilder::offset(distance)` applies an outward offset before shell generation — useful for creating molds where the cavity matches an inflated version of the part (accounting for shrinkage).

The depth pass covers: the shell topology guarantees (always watertight under successful generation; rim closes any open boundary), the cases where shell generation fails (non-watertight input, geometry below the wall-thickness limit), the interaction with `mesh-printability` (shells should round-trip through the printability validator after generation), and the mold-relevance — see below.

## `mesh-shell` and the casting domain

The casting book's [Part 2 Mold Geometry](../../casting_architecture/src/20-mold-geometry.md) discusses mold-cavity generation. The same `ShellBuilder` machinery is the natural foundation:

- **Mold cavity = inverse shell.** A mold is essentially "the negative space around a part, contained within an enclosure." `ShellBuilder` already produces that geometry — wall_thickness becomes mold wall, the inner surface becomes the cavity, the rim becomes the enclosure boundary.
- **Multi-shot sequencing = sequential shells.** Each shot's mold has the previous shot's cured cast as a feature of its cavity — `ShellBuilder` combined with the design SDF produces this directly when the mold cavity SDF is `DifferenceSdf(enclosure, current_cast_state)`.
- **Sim-cast::MoldBuilder** is the casting-domain wrapper that bolts onto `ShellBuilder` with mold-specific concerns (parting surfaces, pour ports, vents, demoldability validation). Same builder pattern, additional manufacturing-concerns layer on top.

The depth pass covers the design pattern transfer in detail and names which `ShellBuilder` features map onto which `MoldBuilder` features.

## `mesh-printability` — print validation

```rust
let config = PrinterConfig::fdm_default();
let result = validate_for_printing(&mesh, &config)?;

if result.is_printable() {
    println!("Pass — {}", result.summary());
} else {
    for issue in result.issues() { /* ... */ }
}
```

Validates a mesh against a `PrinterConfig` describing printer capabilities:

- **Print technology** — FDM, SLA, SLS, MJF, each with characteristic constraints.
- **Minimum wall thickness** — technology-dependent (FDM ~1mm, SLA ~0.4mm, SLS ~0.7mm, MJF ~0.5mm).
- **Maximum overhang angle** — FDM ~45°; SLA more conservative; SLS/MJF unrestricted (powder support).
- **Bridge span limit** — FDM only; how far the printer can bridge unsupported.

Returns a `PrintValidation` enumerating `PrintIssue`s — each with severity, location, and a description. Severity ranges from "warning" (will print but with cosmetic issues) to "fatal" (won't print at all).

Auxiliary tools:

- **`find_optimal_orientation`** — search over orientations to minimize overhang area.
- **`apply_orientation` / `place_on_build_plate`** — apply the chosen orientation to the mesh.
- **Region detection** — `OverhangRegion`, `SupportRegion`, `ThinWallRegion` types that subset a mesh into manufacturability-relevant regions.

The depth pass covers: the orientation search algorithm (axis-aligned vs. SO(3) sampling), the interaction with `mesh-shell` (a generated shell should re-validate as printable), the per-technology constraint sets in detail, and the open question of what to do when validation fails (downstream of validation, the platform's options are: warn the user, attempt automatic fix, refuse to proceed).

## Pattern transfer to casting

The casting domain's [Part 5 Manufacturing Constraints](../../casting_architecture/src/50-constraints.md) will mirror `mesh-printability`'s shape:

```rust
// Casting analogue (proposed; not yet implemented)
let config = CastingProcessConfig::ecoflex_open_pour();
let result = validate_for_casting(&mesh, &cast_sequence, &config)?;
// Returns CastingValidation with CastingIssue list
```

Same structural pattern: `Config` → validation → `Issue` list. Different concerns (draft angles, demoldability, multi-shot adhesion vs. overhang, wall thickness, bridge spans) but identical API shape. The depth pass discusses the transfer in detail and names where `mesh-printability`'s code can be reused (the geometric primitives — manifold checks, region extraction) versus where casting needs its own (draft-angle analysis, demoldability classification).

## What's NOT in these crates

- **Mold-fabrication-specific machinery.** Parting surfaces, pour-port placement, multi-shot sequencing — these are casting-domain concerns. `mesh-shell` produces the geometric primitive (a shell); `sim-cast` will apply mold-specific operations on top.
- **Slicing for layered manufacturing.** Cross-section extraction for FDM/SLA layer-by-layer slicing; out of scope. (Adjacent: `mesh-measure::cross_section` produces single slices, but the slicing-pipeline machinery isn't in mesh-printability.)
- **Print-time prediction / cost estimation.** Estimating how long a part takes to print, or material cost — out of scope; downstream tooling concern.
- **Support structure generation.** `mesh-printability` *detects* regions needing support but doesn't generate the support geometry. Support generation is a separate research-and-engineering problem (and arguably a `mesh-support` crate, if needed).
