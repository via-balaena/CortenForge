# CortenForge Repository Structure

> Produced by the repo audit on 2026-04-11. Replaces the draft skeleton stub.

## Top-Level Map

```
cortenforge/
├── sim/
│   ├── L0/                # Layer 0 — Bevy-free simulation crates
│   │   ├── core/          # Physics engine (90K LOC)
│   │   ├── thermostat/    # Langevin + thermo-computing (Phases 1-6, D1, D2)
│   │   ├── ml-chassis/    # Algorithm chassis: traits, VecEnv, networks, autograd
│   │   ├── rl/            # Generic RL baselines (CEM, PPO, TD3, SAC, REINFORCE)
│   │   ├── opt/           # Gradient-free optimization (SA, richer-SA, PT)
│   │   ├── gpu/           # GPU physics pipeline (wgpu compute)
│   │   ├── mjcf/          # MJCF parser
│   │   ├── urdf/          # URDF parser
│   │   ├── types/         # Shared types
│   │   ├── simd/          # SIMD math
│   │   └── tests/         # Cross-crate conformance tests
│   ├── L1/                # Layer 1 — Bevy integration
│   │   └── bevy/          # sim-bevy (18 crate examples)
│   └── docs/              # Sim-domain docs, specs, todos
│       └── todo/          # Phase specs (7-13), deferred items, v1.0 tracker
├── design/
│   └── cf-design/         # SDF design + mechanisms (Phases 1-5)
├── cf-geometry/           # Geometry primitives (foundation crate)
├── cf-spatial/            # Spatial indexing
├── mesh/                  # Mesh processing umbrella
│   ├── mesh-types/        # Foundation types
│   ├── mesh-io/           # Format I/O (STL, PLY, STEP)
│   ├── mesh-repair/       # Mesh repair
│   ├── mesh-sdf/          # Signed distance fields
│   ├── mesh-offset/       # Mesh offsetting
│   ├── mesh-shell/        # Shell generation
│   ├── mesh-measure/      # Measurement
│   ├── mesh-printability/  # Print validation
│   └── mesh-lattice/      # Lattice generation
├── examples/              # Visual examples (212 total)
│   ├── fundamentals/
│   │   ├── sim-cpu/       # 151 physics examples (25 categories)
│   │   ├── sim-ml/        # 18 ML examples
│   │   ├── design/        # 3 design examples
│   │   └── mesh/          # 1 mesh pipeline example
│   ├── sdf-physics/
│   │   ├── cpu/           # 16 SDF-CPU examples
│   │   └── gpu/           # 1 GPU example
│   └── integration/       # 4 cross-domain examples
├── docs/                  # Cross-domain docs
│   ├── thermo_computing/  # Thermo initiative (vision + foundations)
│   ├── SKELETON.md        # This file
│   ├── STANDARDS.md       # A-grade quality rubric (7 criteria)
│   ├── LAYER_1_SPEC.md    # Current work — visual foundations
│   ├── V1_ROADMAP.md      # v1.0 release path
│   ├── VISION.md          # Platform vision
│   └── INFRASTRUCTURE.md  # Quality gates
├── xtask/                 # Build tooling (check, grade, complete)
└── CLAUDE.md              # AI instructions
```

## Domain Status (2026-04-11)

### sim/L0/core — Physics Engine
- **Status:** Phases 1-13 spec'd. Phases 1-6 implemented. Phases 7-13 drafted.
- **Tests:** ~54 `#[ignore]` tests (5 known blockers, 25+ conformance gaps, rest release-only)
- **Specs:** 7 active docs in `sim/docs/`, 90 phase specs in `sim/docs/todo/spec_fleshouts/`
- **Deferred items:** 119 tracked (48 done, 8 medium → `sim/docs/todo/V1_DEFERRED_ITEMS.md`, 58 low)

### sim/L0/thermostat — Thermo-Computing
- **Status:** Phases 1-6 + D1 + D2 complete. D4 deferred (needs Layers 1-6).
- **Grade:** 97.6% coverage (A+). Completion certification pending.
- **Tests:** 18 `#[ignore]` tests (all legitimate: Gate B verification + release-only)
- **Docs:** Best-organized initiative — `docs/thermo_computing/` (54+ files)

### sim/L0/ml-chassis + rl + opt — ML Stack (3-crate split)
- **Status:** Split from former `sim-ml-bridge` in PR #192 (`2835b4f4`, 2026-04-15).
- **sim-ml-chassis:** Algorithm trait, VecEnv, Policy, Competition, networks, autograd, stats.
- **sim-rl:** 5 generic RL baselines (CEM, REINFORCE, PPO, TD3, SAC) on the chassis.
- **sim-opt:** Gradient-free optimization (SA, richer-SA, PT) + dual-metric rematch analysis.
- **Tests:** 13 `#[ignore]` competition tests + 3 `#[ignore]` D2c-SR rematch fixtures (~4h each).

### sim/L1/bevy — Bevy Integration
- **Status:** All 6 API changes done (accessors, validation harness, sensor viz, HUD, multi-scene).
- **Examples:** 18 crate-internal examples

### design/cf-design — SDF Design Kernel
- **Status:** Phases 1-5 complete. 656 tests.
- **Tests:** 2 `#[ignore]` benchmarks (opt-in)
- **Note:** Depends on sim-core (intentional — design objects become physics objects)

### mesh — Mesh Processing
- **Status:** 9 crates. 0/9 achieve A-grade (coverage + dependency failures).
- **Docs:** Grade tool regrade results documented

### cf-geometry — Geometry Foundation
- **Status:** Universal foundation crate. Used by design, mesh, sim, gpu.
- **Stale refs:** ~15 references to removed `route-*` crates in CF_GEOMETRY_SPEC.md

### xtask — Build Tooling
- **Status:** Fully rebuilt 2026-04-10 (grade + complete commands). Isolated (no workspace deps).

## Dependency Graph

```
cf-geometry (foundation — no workspace deps)
    ├── cf-spatial
    ├── cf-design ────── sim-core
    ├── mesh-types ──── mesh-* chain
    │       └── mesh-io ── sim-mjcf ── sim-urdf
    ├── sim-gpu
    └── sim-core ─────── sim-types + sim-simd
                              │
         sim-bevy ◄────── all L0 crates
              │
         ALL EXAMPLES
```

Zero circular dependencies. Clean L0/L1 separation.

## Examples Inventory (212 total)

| Category | Count | README % | Notes |
|----------|-------|----------|-------|
| sim-cpu fundamentals | 151 | ~95% | 25 subcategories, zero stubs |
| sim-ml fundamentals | 18 | ~85% | PPO/SAC/TD3 lack READMEs |
| design fundamentals | 3 | partial | |
| mesh fundamentals | 1 | partial | |
| SDF-physics cpu | 16 | via EXPECTED_BEHAVIOR | |
| SDF-physics gpu | 1 | no | |
| integration | 4 | no | |
| sim-bevy crate | 18 | no (crate-internal) | |

## Branch Inventory (16 branches)

| Status | Count | Notable |
|--------|-------|---------|
| Active | 10 | thermo-phase-2-3 (26 ahead, current), thermo-doc-review (76 ahead) |
| Ready for PR | 2 | best-policy-tracking (8 ahead), policy-persistence (17 ahead) |
| Stale | 2 | phase-6c, composites-examples |
| Dead (dependabot) | 2 | rand-0.9.2, wgpu-29.0.1 |

## Hygiene Items

- [ ] Delete 2 dead dependabot branches
- [ ] Push feature/thermo-phase-2-3 to remote
- [ ] Merge or close ready-for-PR branches (best-policy-tracking, policy-persistence)
- [ ] Complete sim-thermostat A-grade certification (Step 11 — running)
- [ ] Remaining cleanup tracked in Layer 1 spec (READMEs, etc.)

## Layer 1-6 Roadmap to D4

> D4 (sim-to-real) needs 6 foundation layers. Each layer is a prerequisite
> for the next. Baby steps, rock-solid, ground up.

### Layer 1 — Visual Foundations
**Goal:** Every physics feature has a polished visual example.
- Visual review of 10 actuator examples (camera, README, rev display — motor done, 2-10 pending)
- Track 1B remaining 6 subdirectories (heightfield, Hill muscle, adhesion, flex, collision pairs, plugins)
- Add READMEs to 25 examples missing them (SDF sequence, integration, ML algorithms)
- Add thermo-computing visual examples (Langevin jitter, Kramers escape, bistable switching)

### Layer 2 — CI & Quality Gates
**Goal:** Automated quality enforcement.
- CI pipeline with `--release --ignored` job for heavy tests
- Per-PR grade regression check
- Coverage tracking dashboard

### Layer 3 — Thermo Visual Examples
**Goal:** D1/D2 results visible, not just numerical.
- Brownian ratchet visualization (D1 current → gif)
- Stochastic resonance visualization (D2 SR curve → animated)
- Langevin thermostat visual demo (thermal jitter)

### Layer 4 — cf-design Bridge
**Goal:** Design objects that simulate correctly.
- SDF-as-physics-shape visual proof
- Design-to-sim pipeline example hardening
- cf-design → sim-thermostat integration path

### Layer 5 — Printable Device
**Goal:** First physical artifact from the pipeline.
- Design → mesh → print workflow validated end-to-end
- Tolerance and printability checks automated
- First mechanical p-bit device geometry

### Layer 6 — Measurement & Calibration
**Goal:** Compare sim predictions to physical measurements.
- Measurement protocol for printed device
- Sim-vs-real comparison framework
- Calibration loop (adjust sim params to match reality)

After Layer 6: D4 (sim-to-real) is no longer aspirational — it has
foundations, visual proof, physical artifacts, and measurement data.
