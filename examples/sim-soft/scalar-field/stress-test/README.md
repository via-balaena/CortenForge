# scalar-field — Stress Test (sim-soft `ScalarField` validation superset)

Headless validation of the sim-soft multi-material spatial-field surface —
sharp-CSG partition and smooth-sigmoid blend — against closed-form,
partition, and topology oracles. No window, no Bevy — self-gating assertions
that abort (exit 101) on any mismatch, so `cargo xtask run-validators` runs it
red-or-green.

Folded from two former per-concept examples (`layered-scalar-field` row 8,
`blended-scalar-field` row 9), each now a module. One domain → one stress-test.
Under Rule B, per-tet library-behaviour correctness lives in the lib's own
tests; these modules are the runnable pipeline demonstrations that spot-check
the real mesher outputs against independent oracles + emit the cf-view artifacts.

## Modules

### `layered` — `LayeredScalarField` (sharp CSG step)
A 3-shell concentric `SphereSdf` partition sampled per-tet at the centroid via
`MaterialField::from_fields`. Reads the mesher's own `mesh.materials()` (via
the public `NeoHookean::mu()` / `lambda()`) and checks each tet carries the
Lamé pair of the shell its centroid lands in (oracle = the `SHELL_LAME`
constants indexed by a geometric radius classification — no library-arithmetic
mirror), plus the **`interface_flags`-all-false contract** — this module never
calls `with_interface_sdf`, the exact complement of `blended`'s positive
flagging — and a centroid-inside-body geometry sanity. Per-tet layer-assignment
CORRECTNESS against the `partition_point` rule is owned by the lib (IV-4,
`sim/L0/soft/tests/sdf_material_tagging.rs`); this module is the pipeline
demonstration. Top-line mesh counts are pinned (III-1 contract); per-shell +
z-slab counts are structural (non-empty), not pinned. Emits the categorical
`material_layer_id` scalar (read off the real mesher output).

### `blended` — `BlendedScalarField` (smooth cubic-Hermite smoothstep)
The same body sphere + bbox as `layered`, but a smoothstep transition between
two Lamé regions (`μ = 5e4→2e5`) across a band `‖p‖ ∈ [0.055, 0.085]`. Reads
the mesher's own `mesh.materials()` (via the public `NeoHookean::mu()` /
`lambda()`) and checks it against **independent** oracles — never a
re-derivation of the blend arithmetic: **bit-exact `s=0`/`s=1` snap** to the
endpoint constants outside the band, and at least one band tet carrying a μ
**strictly between** the endpoints (graded, not stepped). The mesher's
`mesh.interface_flags()` are checked to be **mixed** (some `true` — the
`with_interface_sdf` band fires — and some `false`). Per-tet blended-material
and `|φ(x_c)| < L_e` book-rule CORRECTNESS is owned by the lib
(`sim/L0/soft/tests/blended_material_composition.rs`); this module is the
pipeline demonstration. Top-line mesh counts are pinned (III-1 contract); finer
per-bucket / z-slab counts are structural (non-empty), not pinned. Emits three
scalars: `interface_flag`, `material_mu`, `smoothstep_weight` (all read off the
real mesher outputs).

The two are complementary, not subsuming: `blended` demonstrates the
`BlendedScalarField::sample` + `with_interface_sdf` composition path (graded
materials + populated interface flags) `layered` cannot reach; `layered`
demonstrates the sharp 3-shell partition + the no-interface-SDF all-false path.

## Run

```
cargo run -p example-scalar-field-stress-test --release
```

Expected: each module prints its anchor-group summary (`Anchor groups (all
assertions exit-0 on success)`) and the binary exits 0 — a clean exit-0 IS the
correctness signal (there is no `PASS` token; a failed assert aborts with 101).

## Visual artifacts

Each module writes a per-tet centroid PLY (a thin `|z| < cell_size/2` z-slab
subset of the 6768 body tets — projecting the shell structure onto a legible
disk) to `out/` for the `cf-viewer` visual-review path (per-vertex scalars
auto-detected and colormapped):

```
cargo run -p cf-viewer --release -- examples/sim-soft/scalar-field/stress-test/out/material_layer_assignment.ply   # layered: categorical material_layer_id (3 concentric rings)
cargo run -p cf-viewer --release -- examples/sim-soft/scalar-field/stress-test/out/material_blend.ply               # blended: interface_flag + material_mu + smoothstep_weight
```

## Cross-references

- **Inventory**: `sim/L0/soft/EXAMPLE_INVENTORY.md` Tier 3 (rows 8–9).
- **Field impls**: `sim/L0/soft/src/field/layered.rs` (`LayeredScalarField` +
  `BlendedScalarField` + `smoothstep`); interface flags in
  `sim/L0/soft/src/mesh/mod.rs` (`interface_flags_from_field`, IV-6 book rule).
- **Book**: Part 7 §02 §00 (layered scalar field), §02 §01 (blended + IV-6
  interface flags).
