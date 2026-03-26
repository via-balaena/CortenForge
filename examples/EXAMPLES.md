# CortenForge Examples

## Directory Structure

```
examples/
  fundamentals/
    design/           Pure cf-design (implicit surfaces, bio-inspired geometry)
    mesh/             Pure mesh-* (repair, lattice, shell, printability)
    sim-cpu/          CPU physics fundamentals (MJCF, joints)
    sim-gpu/          GPU physics fundamentals (future)
  integration/        Cross-domain pipelines (design → sim → mesh → print)
  sdf-physics/
    cpu/              SDF collision proof ladder (CPU pipeline, 16 baby steps)
    gpu/              GPU physics demos (hockey, future: batch-rl, vr)
```

## fundamentals/design/

Pure `cf-design` examples — implicit surfaces, no physics backend.

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `hello-solid` | `example-hello-solid` | Working | Implicit surface basics |
| `bio-shapes` | `example-bio-shapes` | Working | Bio-inspired geometry library |
| `finger-design` | `example-finger-design` | Needs work | cf-design + sim visualization (dynamics not tuned) |

## fundamentals/mesh/

Pure `mesh-*` examples — mesh processing, no physics.

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `mesh-pipeline` | `example-mesh-pipeline` | Working | Mesh repair, lattice, shell, printability |

## fundamentals/sim-cpu/

CPU physics fundamentals — joints, MJCF, sim-core.

| Example | Package | Status | Notes |
|---------|---------|--------|-------|
| `pendulum-sim` | `example-pendulum-sim` | Working | Raw MJCF, pure physics (no cf-design) |

## integration/

Pipeline composition: proving domains work together end-to-end.
All use `Mechanism::to_model()` (SDF collision + visual mesh, no MJCF round-trip).

| Example | Package | Pipeline | Status |
|---------|---------|----------|--------|
| `design-to-sim` | `example-design-to-sim` | design → simulate → visualize | Working |
| `design-to-print` | `example-design-to-print` | design → mesh → print | Working |
| `sim-informed-design` | `example-sim-informed-design` | design → simulate → stress-graded lattice | Working |
| `full-pipeline` | `example-full-pipeline` | design → simulate → stress → lattice → print | Working |

## sdf-physics/cpu/

Baby-step ladder proving the SDF collision thesis on the CPU pipeline.
**One new variable per step.** Each step depends on every previous step working.

| Step | Example | Package | What it proves | Status |
|------|---------|---------|---------------|--------|
| 01 | `01-sdf-grid` | `example-sdf-cpu-01-sdf-grid` | SdfGrid from solid sphere | Working |
| 02 | `02-thin-grid` | `example-sdf-cpu-02-thin-grid` | Thin-wall grid fidelity | Working |
| 03 | `03-freefall` | `example-sdf-cpu-03-freefall` | to_model() + gravity | Working |
| 04 | `04-rest` | `example-sdf-cpu-04-rest` | sdf_plane_contact | Stub |
| 05 | `05-drop` | `example-sdf-cpu-05-drop` | Dynamic contact + restitution | Stub |
| 06 | `06-slide` | `example-sdf-cpu-06-slide` | Friction / tangential forces | Stub |
| 07 | `07-pair` | `example-sdf-cpu-07-pair` | sdf_sdf_contact | Stub |
| 08 | `08-stack` | `example-sdf-cpu-08-stack` | Multi-body stacking | Stub |
| 09 | `09-cube-in-box` | `example-sdf-cpu-09-cube-in-box` | Concave containment | Stub |
| 10 | `10-ball-in-bowl` | `example-sdf-cpu-10-ball-in-bowl` | Curved concave SDF contact | Stub |
| 11 | `11-hinge-free` | `example-sdf-cpu-11-hinge-free` | Joints with SDF bodies | Stub |
| 12 | `12-hinge-wall` | `example-sdf-cpu-12-hinge-wall` | Articulated external contact | Stub |
| 13 | `13-hinge-stop` | `example-sdf-cpu-13-hinge-stop` | Parent-child SDF contact (convex) | Stub |
| 14 | `14-damped-hinge` | `example-sdf-cpu-14-damped-hinge` | Damping + parameter sensitivity | Stub |
| 15 | `15-concave-stop` | `example-sdf-cpu-15-concave-stop` | Concave parent-child contact | Stub |
| 16 | `16-socket` | `example-sdf-cpu-16-socket` | Full socket/condyle | Blocked |

## sdf-physics/gpu/

GPU physics demos — full wgpu compute pipeline (broadphase → narrowphase → solver → integration).
Free joints only, nv ≤ 60.

| Step | Example | Package | What it proves | Status |
|------|---------|---------|---------------|--------|
| 01 | `01-hockey` | `example-sdf-gpu-01-hockey` | Actuated impact, momentum transfer via GPU SDF | In progress |

## Key Architecture: `Mechanism::to_model()`

`cf-design` can build a `sim-core::Model` directly from a `Mechanism`,
bypassing the MJCF XML round-trip. Each part produces two geoms:

- **SDF geom** (collision): `SdfGrid` from the implicit surface — O(1)
  distance queries with exact distance semantics.
- **Mesh geom** (visual): triangle mesh from the same `Solid` — high-res
  rendering in Bevy.

Both derive from the identical mathematical object. What you see is what
physics simulates.
