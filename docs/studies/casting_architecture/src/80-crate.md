# Crate architecture sketch

What this part covers (when authored to depth):

## Crate name

`sim-cast` (working title). Sibling to `sim-soft`. Lives at `sim/L0/cast/` if it follows the existing tier convention, or `sim/L1/cast/` if it depends on `sim-soft` (which it will).

## Module layout

Mirroring `sim-soft`'s shape:

```
sim/L?/cast/src/
├── lib.rs
├── materials/          # cast-side material database (Ecoflex, etc.)
│   └── mod.rs
├── mold/               # mold geometry generation
│   ├── mod.rs
│   ├── cavity_inversion.rs
│   ├── enclosure.rs
│   └── flow_features.rs   # ports, vents, runners, risers
├── parting/            # parting strategy and demoldability
│   ├── mod.rs
│   ├── parting_surface.rs
│   ├── demoldability.rs
│   └── undercut.rs
├── sequence/           # multi-shot sequencing
│   ├── mod.rs
│   ├── ordering.rs
│   ├── adhesion.rs
│   └── mold_in_mold.rs
├── constraints/        # manufacturability checks
│   ├── mod.rs
│   ├── wall_thickness.rs
│   ├── draft.rs
│   └── cost.rs
├── forward/            # ForwardManufacturability (mirrors sim-soft::ForwardMap)
│   └── mod.rs
├── output/             # STL/STEP/JSON/Markdown emission
│   ├── mod.rs
│   ├── stl.rs
│   ├── step.rs
│   └── docs.rs
└── differentiable/     # gradients through manufacturability checks
    └── mod.rs
```

## Trait surfaces

The shape parallels `sim-soft`:

- `Sdf` — re-exported from `sim-soft::sdf_bridge`, not redefined. Same trait, both crates consume it.
- `MoldGeometry` — emits `(cavity_sdf, port_positions, vent_positions, core_specs)` from a part SDF.
- `PartingStrategy` — emits parting surfaces and demoldability scores.
- `CastSequence` — emits ordered shots from a part + `MaterialField`.
- `ManufacturabilityCheck` — evaluates constraints, emits a `ManufacturabilityReport`.
- `ForwardManufacturability` — γ-locked ForwardMap-style API for design-loop integration.
- `Differentiable` — VJP registry for differentiable manufacturability terms (where defined).

## Coupling boundaries

- **With `sim-soft`.** The big one. Same `Sdf` trait, same `MaterialField`. Casting reads what the simulator reads.
- **With `cf-design`.** The design authoring layer feeds both `sim-soft` and `sim-cast`. Edits propagate to both; manufacturability constraints surface back into the editor as warnings or live overlays.
- **With `sim-bevy`.** Mold geometry visualization. Probably reuses the same PLY-based pipeline the soft-body examples directory establishes.
- **With `sim-ml-chassis`.** Same `Tape` / `Tensor` types. `ForwardManufacturability` writes to the same tape `ForwardMap` writes to (or computes its own and adds at reward level — open question per [Part 6](60-loop-integration.md)).
- **With `mesh-io`.** STL / STEP / 3MF emission lives here, possibly reused.

## Build order

A walking-skeleton phase plan, mirroring the soft-body book's approach:

1. **Skeleton.** Single concrete pipeline: take a `SphereSdf`, emit a two-piece planar-parted mold STL. No multi-shot, no manufacturability checks, no design-loop integration. Walking-skeleton invariants (the cast STL is a closed surface, the cavity SDF correctly inverts the part SDF, etc.).
2. **Mold geometry depth.** Pour ports, vents, cores. Demoldability for planar parting. Wall-thickness check.
3. **Multi-shot.** Sequencing for nested-shell parts. Inter-shot adhesion strategies. Layered silicone device as the canonical multi-shot test.
4. **Forward manufacturability.** `ForwardManufacturability` trait wired to `RewardBreakdown`; sim-cast becomes part of the optimizer's reward computation.
5. **Free-form parting.** Algorithmic parting-surface generation for non-planar geometries. Research-frontier work.
6. **Sim-to-real for casting.** Residual GP over cast-side measurements (shrinkage, bubble incidence, cure variation).

## Open questions

- Tier placement (L0 vs L1). Depends on whether `sim-cast` directly depends on `sim-soft` types. Probably yes (re-exports `Sdf`, `MaterialField`), so L1.
- Optional vs required for the design loop. Should the loop run with `sim-cast` always-on, or as an opt-in? Probably always-on once skeleton lands; the manufacturability reward term defaults to zero for unconstrained designs and only bites when constraints are set.
