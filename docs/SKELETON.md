# CortenForge Repository Skeleton (DRAFT)

> Stub for the repo audit. Not prescriptive yet — just enough structure
> to guide the audit session. Revise after the audit produces real data.

## Top-level map

```
cortenforge/
├── sim/
│   ├── L0/           # Bevy-free simulation crates
│   │   ├── core/     # Physics engine
│   │   ├── thermostat/# Langevin + thermo-computing
│   │   ├── ml-bridge/ # RL algorithms + autograd
│   │   ├── gpu/      # GPU physics pipeline
│   │   ├── mjcf/     # MJCF parser
│   │   ├── urdf/     # URDF parser
│   │   ├── types/    # Shared types
│   │   ├── simd/     # SIMD math
│   │   └── tests/    # Cross-crate conformance tests
│   ├── L1/           # Bevy integration (sim-bevy)
│   └── docs/         # Sim-domain docs + todos
├── cf-design/        # SDF design + mechanisms
├── cf-geometry/      # Geometry primitives
├── cf-spatial/       # Spatial indexing
├── mesh/             # Mesh processing
├── examples/         # Visual examples (all domains)
├── docs/             # Cross-domain docs
│   ├── thermo_computing/  # Thermo initiative (well-organized)
│   └── ???           # Other initiatives need structure
├── xtask/            # Build tooling (check, grade)
├── CLAUDE.md         # AI instructions
└── SKELETON.md       # This file (temporary)
```

## Per-domain pattern (proposed)

Each domain should follow the same shape:

```
domain/
├── src/              # Code
├── tests/            # Integration tests
├── docs/             # Domain docs
│   ├── STATUS.md     # Current state, what's done, what's next
│   ├── specs/        # Active specs
│   └── findings/     # Research results (like D1d, D2c)
├── examples/         # Domain examples (or in top-level examples/)
└── README.md         # One-paragraph orientation
```

## Status tracking (proposed)

One file per domain that answers "where are we?":

- `sim/L0/thermostat/docs/STATUS.md` — Phases 1-6 done, D1 done, D2 done, D4 next
- `sim/L0/ml-bridge/docs/STATUS.md` — 5 algorithms, autograd, policy persistence
- `sim/L1/docs/STATUS.md` — API changes 1-3 done, Change 4 pending
- `cf-design/docs/STATUS.md` — Phases 1-5 done
- `examples/STATUS.md` — coverage %, what's built, what's pending

## Examples organization (proposed)

```
examples/
├── STATUS.md              # Coverage map
├── fundamentals/          # Per-physics-concept demos
│   ├── sim-cpu/           # CPU physics examples
│   │   ├── sensors/
│   │   ├── actuators/
│   │   ├── constraints/
│   │   └── thermo/        # ← NEW: thermo-computing visuals
│   └── sim-gpu/           # GPU examples
├── design/                # cf-design examples
├── integration/           # Cross-domain (design → sim, sim → ml)
└── research/              # D1, D2, D4 visual demos
```

## Audit checklist (for next session)

- [ ] Inventory every example — what exists, what compiles, what's orphaned
- [ ] Inventory every spec — active vs archived vs stale
- [ ] Inventory every `#[ignore]` test — last run date, still valid?
- [ ] Inventory open branches — merged? diverged? abandoned?
- [ ] Inventory docs — where's the duplication? what's outdated?
- [ ] Map cross-domain dependencies — what touches what?
- [ ] Identify the "Layer 1 → Layer 6" path with concrete deliverables
