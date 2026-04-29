# Mesh Examples

Focused, single-concept demonstrations of the 10-crate mesh ecosystem
(`mesh-types`, `mesh-io`, `mesh-repair`, `mesh-sdf`, `mesh-offset`,
`mesh-shell`, `mesh-measure`, `mesh-printability`, `mesh-lattice`, plus
the `mesh` umbrella).

Each example writes one or more PLY artifacts to its own `out/`
directory; open the artifact externally (ParaView, MeshLab, Blender) for
the visuals pass. The architecture book at
[`docs/studies/mesh_architecture/`](../../docs/studies/mesh_architecture/)
covers design rationale; this directory is the executable proof.

For a cross-domain Bevy walkthrough that strings together repair → shell
→ lattice → printability with side-by-side scene rendering, see
[`examples/fundamentals/mesh/mesh-pipeline`](../fundamentals/mesh/mesh-pipeline).
That example complements (does not replace) the focused single-concept
examples here.

## Examples

| Example | Concept |
|---------|---------|
| [`ply-with-custom-attributes`](ply-with-custom-attributes/) | Per-vertex custom-attribute PLY round-trip via `save_ply_attributed` |
| [`format-conversion`](format-conversion/) | STL ↔ PLY ↔ OBJ — what each format preserves (geometry vs topology) |
| [`attributed-mesh-basics`](attributed-mesh-basics/) | The SoA attributed-mesh shape — built-in slots, extras, length validation (no I/O) |
| [`mesh-repair-walkthrough`](mesh-repair-walkthrough/) | The repair pipeline pattern from broken to clean (validate → repair → fill → re-wind → re-validate) |

## Tier 1 — first to land (validates new infrastructure)

| Example | Concept |
|---------|---------|
| [`ply-with-custom-attributes`](ply-with-custom-attributes/) | Per-vertex custom-attribute PLY round-trip via `save_ply_attributed` |

## Tier 2 — alongside soft-body, foundation for sim-cast

| Example | Concept |
|---------|---------|
| [`format-conversion`](format-conversion/) | STL ↔ PLY ↔ OBJ — what each format preserves (geometry vs topology) |
| [`attributed-mesh-basics`](attributed-mesh-basics/) | The SoA attributed-mesh shape (no I/O) |
| [`mesh-repair-walkthrough`](mesh-repair-walkthrough/) | validate → repair → re-validate flow |
| `mesh-offset-outward` | SDF-based outward offset (expansion) |
| `mesh-offset-inward` | SDF-based inward offset (cavity prep) |
| `shell-generation-fast` | Normal-based shell (varying wall thickness) |
| `shell-generation-high-quality` | SDF-based shell (uniform wall thickness) |
| `printability-fdm-validation` | Manufacturability constraint reporting |
| `printability-orientation` | Auto-orientation reduces overhang |
| `cross-section-sweep` | Slice-stack PLY contours along an axis |

## Tier 3 — deferred to a later PR

5 examples covering numerical SDF visualization, lattice generation, and
self-intersection detection. See architecture book Part 8 for the full
inventory; these land when their underlying use cases (lattice work for
the layered silicone device, sdf-distance work for casting demoldability
analysis) drive them.

## Layout convention

Every example is a workspace member crate at:

```
examples/mesh/<name>/
├── Cargo.toml     # [package].name = "example-mesh-<name>"
├── README.md      # museum-plaque (template below)
├── src/main.rs    # writes PLY to out/
└── out/           # gitignored; generated artifacts
```

Names are dash-case to match the rest of the workspace
(`mesh-pipeline`, `hello-solid`, etc.).

## Per-example README template

````markdown
# <example name>

<one-paragraph concept statement — the platform capability this demonstrates>

## What it does

<2–4 sentences: input mesh, transformation applied, output written>

## Numerical anchors

- <verifiable number Claude checks (vertex count, manifold check, volume, …)>
- <…>

## Visuals

- <what the user looks for in the PLY (shape, color gradient, …)>
- <…>

## Run

```
cargo run -p example-mesh-<name> --release
```

Output written to `out/<filename>.ply`.
````

## Cadence

Two-pass review per example (per `feedback_one_at_a_time` and
`feedback_one_at_a_time_review`):

1. **Numbers pass (Claude)** — runs the example, verifies the numerical
   anchors listed in the example's README.
2. **Visuals pass (user)** — opens the PLY in an external viewer,
   confirms the visual matches expectations.

Examples are reviewed individually before the next one lands. Multiple
examples bundle into one PR per `feedback_pr_size_ci_economics`.
