# Why this study exists

|  |  |
|---|---|
| **What** | A study in casting/molding as the manufacturing path from a CortenForge-authored soft-body design to a physical part. |
| **Why** | The soft-body architecture book stops at the simulation surface. Real parts get manufactured. For silicones, polyurethanes, and similar elastomers, casting is the dominant fab path — and it has its own design surface (mold geometry, parting, sequencing, demoldability) that bolts onto the same SDF foundation the simulator uses. |
| **Goal** | Build the integrated version: given an SDF-authored soft-body design with a `MaterialField`, generate a manufacturable mold (or sequence of molds), surface manufacturability constraints back to the design loop, and emit fab-ready output (STL/STEP for the mold, casting sequence docs, BOM). |
| **Canonical test** | The layered silicone device that the soft-body architecture book targets. Three concentric shells of distinct silicone formulations, hollow cavity, sanitized geometry. Castable in three shots through three topologically distinct molds. |
| **Core thesis** | The simulator's geometry IS the manufacturing geometry. The same SDF that defines the part defines the cavity that defines the mold. Manufacturability isn't a downstream check — it's a constraint that participates in the design optimization. |

## Scope

This book covers casting/molding as a fab path. It does NOT cover:

- 3D printing — the soft-body book's [Part 10 active learning](../../soft_body_architecture/src/100-optimization/04-active-learning.md) handles "what to print next" for additive paths.
- CNC machining of finished parts — silicones don't machine.
- Injection molding — high-volume manufacturing path with fundamentally different mold-design constraints; out of scope for the bench-scale soft-body work CortenForge targets.
- Mold *fabrication* itself — once we emit STL/STEP, the actual cutting / printing of the mold is somebody else's problem (CNC shop, FDM/SLA printer).

## Relationship to the soft-body book

This is a sibling study. The soft-body architecture book at `docs/studies/soft_body_architecture/` is the canonical spec for simulation; this book is the canonical spec for the casting fab path. They cross-reference at:

- **SDF surface.** Both books use the same `Sdf` trait surface from `sim-soft::sdf_bridge`. Casting reads it; simulation tetrahedralizes it.
- **`MaterialField`.** Multi-material casting sequences map directly to the material partition the simulator already consumes.
- **Reward composition.** The soft-body book's [`RewardBreakdown`](../../soft_body_architecture/src/100-optimization/00-forward.md) gains a manufacturability term sourced from this domain.
- **Coupling boundary.** [Soft-body Part 11 Ch 02](../../soft_body_architecture/src/110-crate/02-coupling.md) names `sim-cast` as a coupled crate.

## Reading order

This study is currently a skeleton. Each part is a one-page framing of what the full chapter will cover, intended to make the architecture's shape legible before the depth pass arrives. Read in numbered order; the dependency graph mirrors the soft-body book's ordering (context → materials → geometry → process → output → integration → roadmap).

## How this study is produced

Same cadence as the soft-body book: deep recon before commitment, two-pass prose review, ceiling-quality at every leaf, no timeline pressure. The skeleton lands during the soft-body examples-directory work to make the casting domain a named neighbor of the simulator from the start; depth-pass authoring waits until the soft-body examples have surfaced concrete observations about what manufacturing constraints actually need to be modeled.
