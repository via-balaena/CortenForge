# Examples Directory Reorganization Spec

**Status:** Stress-tested — ready for review
**Date:** 2026-03-25
**Branch:** `examples`

## Motivation

The GPU physics pipeline (sim-gpu) is now implemented. Examples need to cover
both CPU and GPU backends. The current flat structure conflates pure-design
examples with physics examples and has no GPU coverage.

## Current Structure

```
examples/
  EXAMPLES.md
  fundamentals/                         # Mixed domains, all CPU
    hello-solid/          example-hello-solid          cf-design only
    bio-shapes/           example-bio-shapes           cf-design only
    mesh-pipeline/        example-mesh-pipeline        mesh-* only
    pendulum-sim/         example-pendulum-sim         sim-core (hinge joints, MJCF)
    finger-design/        example-finger-design        cf-design + sim-core
  integration/                          # Cross-domain pipelines
    design-to-sim/        example-design-to-sim
    design-to-print/      example-design-to-print
    sim-informed-design/  example-sim-informed-design
    full-pipeline/        example-full-pipeline
  sdf-physics/                          # CPU proof ladder (01–16)
    01-sdf-grid/          example-sdf-01-sdf-grid
    ...
    16-socket/            example-sdf-16-socket
```

## Target Structure

```
examples/
  EXAMPLES.md                           # Rewritten index
  fundamentals/
    design/                             # Pure cf-design (no physics)
      hello-solid/        example-hello-solid
      bio-shapes/         example-bio-shapes
      finger-design/      example-finger-design
    mesh/                               # Pure mesh-* (no physics)
      mesh-pipeline/      example-mesh-pipeline
    sim-cpu/                            # CPU physics fundamentals
      pendulum-sim/       example-pendulum-sim
    sim-gpu/                            # GPU physics fundamentals
      (empty for now — future: hello-gpu)
  integration/                          # Cross-domain pipelines (unchanged)
    design-to-sim/        example-design-to-sim
    design-to-print/      example-design-to-print
    sim-informed-design/  example-sim-informed-design
    full-pipeline/        example-full-pipeline
  sdf-physics/
    cpu/                                # Existing CPU proof ladder
      01-sdf-grid/        example-sdf-cpu-01-sdf-grid
      02-thin-grid/       example-sdf-cpu-02-thin-grid
      03-freefall/        example-sdf-cpu-03-freefall
      04-rest/            example-sdf-cpu-04-rest
      05-drop/            example-sdf-cpu-05-drop
      06-slide/           example-sdf-cpu-06-slide
      07-pair/            example-sdf-cpu-07-pair
      08-stack/           example-sdf-cpu-08-stack
      09-cube-in-box/     example-sdf-cpu-09-cube-in-box
      10-ball-in-bowl/    example-sdf-cpu-10-ball-in-bowl
      11-hinge-free/      example-sdf-cpu-11-hinge-free
      12-hinge-wall/      example-sdf-cpu-12-hinge-wall
      13-hinge-stop/      example-sdf-cpu-13-hinge-stop
      14-damped-hinge/    example-sdf-cpu-14-damped-hinge
      15-concave-stop/    example-sdf-cpu-15-concave-stop
      16-socket/          example-sdf-cpu-16-socket
    gpu/                                # GPU physics demos
      01-hockey/          example-sdf-gpu-01-hockey
      (future: 00-freefall, batch-rl, vr-hockey)
```

## Naming Convention

Package names encode the full path for `cargo run -p`:

| Category | Pattern | Example |
|----------|---------|---------|
| Fundamentals (design) | `example-{name}` | `example-hello-solid` |
| Fundamentals (mesh) | `example-{name}` | `example-mesh-pipeline` |
| Fundamentals (sim-cpu) | `example-{name}` | `example-pendulum-sim` |
| Integration | `example-{name}` | `example-design-to-sim` |
| SDF CPU | `example-sdf-cpu-{NN}-{name}` | `example-sdf-cpu-08-stack` |
| SDF GPU | `example-sdf-gpu-{NN}-{name}` | `example-sdf-gpu-01-hockey` |

**Fundamentals and integration names stay unchanged** — no churn on names
that work fine. Only the sdf-physics examples get `cpu`/`gpu` prefixes since
they're the ones that have backend variants.

## Changes Required

### 1. Directory moves (fundamentals)

| From | To |
|------|-----|
| `fundamentals/hello-solid/` | `fundamentals/design/hello-solid/` |
| `fundamentals/bio-shapes/` | `fundamentals/design/bio-shapes/` |
| `fundamentals/finger-design/` | `fundamentals/design/finger-design/` |
| `fundamentals/mesh-pipeline/` | `fundamentals/mesh/mesh-pipeline/` |
| `fundamentals/pendulum-sim/` | `fundamentals/sim-cpu/pendulum-sim/` |

**No Cargo.toml name changes** for fundamentals — only paths change.

### 2. Directory moves (sdf-physics)

| From | To |
|------|-----|
| `sdf-physics/01-sdf-grid/` | `sdf-physics/cpu/01-sdf-grid/` |
| `sdf-physics/02-thin-grid/` | `sdf-physics/cpu/02-thin-grid/` |
| `sdf-physics/03-freefall/` | `sdf-physics/cpu/03-freefall/` |
| `sdf-physics/04-rest/` | `sdf-physics/cpu/04-rest/` |
| `sdf-physics/05-drop/` | `sdf-physics/cpu/05-drop/` |
| `sdf-physics/06-slide/` | `sdf-physics/cpu/06-slide/` |
| `sdf-physics/07-pair/` | `sdf-physics/cpu/07-pair/` |
| `sdf-physics/08-stack/` | `sdf-physics/cpu/08-stack/` |
| `sdf-physics/09-cube-in-box/` | `sdf-physics/cpu/09-cube-in-box/` |
| `sdf-physics/10-ball-in-bowl/` | `sdf-physics/cpu/10-ball-in-bowl/` |
| `sdf-physics/11-hinge-free/` | `sdf-physics/cpu/11-hinge-free/` |
| `sdf-physics/12-hinge-wall/` | `sdf-physics/cpu/12-hinge-wall/` |
| `sdf-physics/13-hinge-stop/` | `sdf-physics/cpu/13-hinge-stop/` |
| `sdf-physics/14-damped-hinge/` | `sdf-physics/cpu/14-damped-hinge/` |
| `sdf-physics/15-concave-stop/` | `sdf-physics/cpu/15-concave-stop/` |
| `sdf-physics/16-socket/` | `sdf-physics/cpu/16-socket/` |
| `sdf-physics/10b-hockey/` | `sdf-physics/gpu/01-hockey/` |

**Cargo.toml name changes** for sdf-physics:
- `example-sdf-{NN}-{name}` → `example-sdf-cpu-{NN}-{name}` (all 16)
- `example-sdf-10b-hockey` → `example-sdf-gpu-01-hockey`

### 3. Workspace Cargo.toml updates

Update **all 22 example member paths** in `[workspace] members`:
- 5 fundamentals paths: insert `design/`, `mesh/`, or `sim-cpu/` subdirectory
- 16 sdf-physics CPU paths: insert `cpu/` subdirectory
- 1 sdf-physics GPU path: `sdf-physics/10b-hockey` → `sdf-physics/gpu/01-hockey`
- 4 integration paths: **unchanged**

### 4. Package name changes (Cargo.toml inside each example)

Only sdf-physics examples get renamed:
- 16 CPU: `example-sdf-{NN}-{name}` → `example-sdf-cpu-{NN}-{name}`
- 1 GPU: `example-sdf-10b-hockey` → `example-sdf-gpu-01-hockey`

Fundamentals and integration package names stay unchanged.

### 5. Doc comment updates (`//! Run with:`)

Update in 17 sdf-physics `main.rs` files (package name changed).
Fundamentals doc comments already use package names (unchanged), no update needed.

### 6. Co-located spec/doc files

These files live inside example directories and move automatically with
`git mv` of the parent directory — no extra handling needed:

| File | Moves with |
|------|-----------|
| `sdf-physics/07-pair/EXPECTED_BEHAVIOR.md` | → `cpu/07-pair/` |
| `sdf-physics/08-stack/ANALYTICAL_CONVEX_SPEC.md` | → `cpu/08-stack/` |
| `sdf-physics/08-stack/EXPECTED_BEHAVIOR.md` | → `cpu/08-stack/` |
| `sdf-physics/10b-hockey/EXPECTED_BEHAVIOR.md` | → `gpu/01-hockey/` |
| `sdf-physics/11-hinge-free/EXPECTED_BEHAVIOR.md` | → `cpu/11-hinge-free/` |
| `sdf-physics/11-hinge-free/POLISH_TODO.md` | → `cpu/11-hinge-free/` |
| `integration/INTEGRATION_SPEC.md` | stays (integration unchanged) |

### 7. Cross-references in domain docs

These files outside `examples/` reference example paths and need updating:

| File | Reference | New value |
|------|-----------|-----------|
| `sim/docs/GPU_PHYSICS_PIPELINE_SPEC.md` | `examples/sdf-physics/10b-hockey/` | `examples/sdf-physics/gpu/01-hockey/` |
| `sim/docs/PYRAMIDAL_FRICTION_INSTABILITY.md` | `examples/sdf-physics/07-pair` | `examples/sdf-physics/cpu/07-pair` |
| `design/cf-design/docs/SDF_NATIVE_PHYSICS_SPEC.md` | `examples/fundamentals/finger-design/` (×2) | `examples/fundamentals/design/finger-design/` |
| `examples/sdf-physics/08-stack/ANALYTICAL_CONVEX_SPEC.md` | `examples/sdf-physics/08-stack` | `examples/sdf-physics/cpu/08-stack` |

### 8. EXAMPLES.md rewrite

Full rewrite to reflect new structure and categories.

## What does NOT change

- **Source code** — no `main.rs` logic changes (except `//! Run with:` comments)
- **Dependencies** — no Cargo.toml dependency changes
- **Integration examples** — paths and names unchanged
- **Fundamental example names** — only paths change, package names stay
- **CI/CD** — workflows use `*/examples/*` glob, depth-agnostic
- **xtask** — safety scan uses `/examples/` substring match, unaffected
- **Cargo.lock** — auto-regenerates from workspace Cargo.toml

## Validation

After the move:
1. `cargo build -p example-hello-solid` (fundamentals — path changed, name same)
2. `cargo build -p example-sdf-cpu-08-stack` (sdf-cpu — path + name changed)
3. `cargo build -p example-sdf-gpu-01-hockey` (sdf-gpu — path + name changed)
4. `cargo build -p example-design-to-sim` (integration — unchanged)
5. Full workspace `cargo check` passes

## Stress Test Results (2026-03-25)

### Passed

- **CI/CD safe**: All 3 workflows use `*/examples/*` or `-not -path "*/examples/*"` —
  depth-agnostic, no changes needed.
- **xtask safe**: Safety scan uses `line.contains("/examples/")` — unaffected.
- **No cross-example dependencies**: Examples don't import each other.
- **finger-design placement confirmed**: 80% cf-design geometry, sim is visualization.
  Belongs in `design/`.

### Gaps found and fixed

1. **Co-located files** (§6): 7 spec/doc files inside example dirs were not mentioned.
   They move automatically with `git mv` — now documented.
2. **Domain doc cross-refs** (§7): 4 files outside `examples/` reference example paths.
   Now enumerated with exact old → new values.
3. **Workspace path count**: Spec now correctly states 22 paths change (not just
   sdf-physics).

### No issues found

- No circular dependencies
- No feature-flag interactions
- No platform-specific path concerns
- No lock file conflicts (auto-regenerates)

## Resolved Questions

1. **`finger-design` placement**: `fundamentals/design/`. Confirmed by code analysis —
   it's 80% cf-design geometry (sockets, knuckles, tendon channels), sim is
   secondary visualization.

2. **`fundamentals/sim-gpu/` empty**: Don't create until first GPU fundamental
   example exists. No empty placeholder directories.

3. **Renumbering 10b-hockey → 01-hockey**: Confirmed. `10b` was CPU-ladder-relative.
   GPU folder starts its own sequence at `01`.
