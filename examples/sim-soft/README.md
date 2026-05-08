# sim-soft Examples

Focused, single-concept demonstrations of the `sim-soft` soft-body
FEM crate (`sim/L0/soft/`) — SDF primitives + meshing, hyperelastic
constitutive laws, multi-element + multi-material assembly, penalty
contact, and the cf-design / mesh-sdf bridges that make the
layered silicone device cavity-fit workflow possible.

Each example writes static artifacts (PLY + JSON) or renders live
in Bevy depending on the capability tier — see "Visualization
convention" below. Per-example READMEs document the locked
numerical anchors (asserted by `cargo run --release` exit-0). The
arc-level inventory at
[`sim/L0/soft/EXAMPLE_INVENTORY.md`](../../sim/L0/soft/EXAMPLE_INVENTORY.md)
covers strategy, ordering, and PR sequencing; this directory is the
executable proof.

## Examples by capability tier

(Filled in commit-by-commit as the arc lands. The inventory file is
the source of truth for what's planned vs shipped.)

### Tier 1 — SDF primitives + meshing

| Example | Concept |
|---------|---------|
| [`sphere-sdf-eval`](sphere-sdf-eval/) | The `Sdf` trait contract on `SphereSdf` — analytic signed distance and unit-length gradient, including the documented `Vec3::z()` origin-singularity fallback; 11³ = 1331 grid sweep emitted as PLY with `extras["signed_distance"]` |
| [`hollow-shell-sdf`](hollow-shell-sdf/) | Sharp-CSG difference combinator `SphereSdf{R_OUTER=1.0} \ SphereSdf{R_CAVITY=0.5}` via `DifferenceSdf` (book Part 7 §00 §01); emits a 2-D z = 0 slice (49² = 2401 verts) with two scalars — `signed_distance` (donut cross-section) and `active_branch` (visualizes the equidistant branch-flip circle at `\|p\| = 0.75`) |

### Tier 5 — Bridges + extensions (silicone-device path)

| Example | Concept |
|---------|---------|
| [`mesh-scan-as-solid`](mesh-scan-as-solid/) | `mesh_sdf::SignedDistanceField` satisfies `cf_design::Sdf` (PR3 F2). 12-tri programmatic cube fixture round-tripped through binary STL on disk; closed-form L∞-ball anchors at face / edge / vertex / interior probes via `&dyn cf_design::Sdf`; 17³ = 4913 bulk grid PLY with two scalars — `signed_distance` (analytical SDF) and `inside_raycast` (raycast inside-test, with documented HE-1 diagonal gaps) |

## Visualization convention

Examples split by tier per
[`EXAMPLE_INVENTORY.md`](../../sim/L0/soft/EXAMPLE_INVENTORY.md)
§Visualization convention:

- **PLY + JSON** for static math-pass-first artifacts (Tier 1-3 +
  Tier 5). Open in [`cf-view`](../../cf-viewer/) — the workspace's
  unified Bevy-CLI viewer with auto-discovered per-vertex scalars
  and auto-colormap detection. Per-example READMEs call out the
  scalar to pre-select and the canonical visual.
- **Bevy real-time** for contact dynamics + the silicone-device
  synthesis (Tier 4 + Tier 6). Headless asserts pre-render +
  visual-mode playback under one `cargo run`. Mirrors the
  `examples/fundamentals/sim-cpu/` convention.

`feedback_visible_contacts` is the hard requirement that lands
contact-tier examples in Bevy. Pure SDF / constitutive / multi-
material / bridge examples are static.

## Layout convention

Every example is a workspace member crate at:

```
examples/sim-soft/<name>/
├── Cargo.toml     # [package].name = "example-sim-soft-<name>"
├── README.md      # museum-plaque (template per `examples/mesh/README.md`)
├── src/main.rs    # writes PLY / JSON to out/, or runs a Bevy app
└── out/           # gitignored; generated artifacts (PLY / JSON)
```

Names are dash-case, matching the rest of the workspace.

## Cadence

Two-pass review per example (per `feedback_one_at_a_time` and
`feedback_one_at_a_time_review`):

1. **Numbers pass (Claude)** — runs the example, verifies the
   numerical anchors. For static-artifact examples authored under
   `feedback_math_pass_first_handauthored`, anchors are
   `assert_relative_eq!` calls in `src/main.rs` and a clean exit-0
   IS the correctness signal.
2. **Visuals pass (user)** — opens the PLY in cf-view (or watches
   the Bevy playback) to confirm the visual matches expectations.

Examples are reviewed individually before the next one lands.
Multiple examples bundle into one PR per
`feedback_pr_size_ci_economics`.
