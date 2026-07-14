# scalar-field — Stress Test (sim-soft `ScalarField` validation superset)

Headless validation of the sim-soft multi-material spatial-field surface —
sharp-CSG partition and smooth-sigmoid blend — against closed-form,
partition, and topology oracles. No window, no Bevy — self-gating assertions
that abort (exit 101) on any mismatch, so `cargo xtask run-validators` runs it
red-or-green.

Folded from two former per-concept examples (`layered-scalar-field` row 8,
`blended-scalar-field` row 9), each now a module preserving its hand-authored
fixture and oracle checks verbatim. One domain → one stress-test.

## Modules

### `layered` — `LayeredScalarField` (sharp CSG step)
A 3-shell concentric `SphereSdf` partition sampled per-tet at the centroid via
`MaterialField::from_fields`. Validates the material-tag partition against a
test-side `partition_point` re-derivation (per-tet `NeoHookean` energy +
`first_piola` bit-equal at `F = diag(1.2, 1, 1)`), exactly **3 unique layer ids
`{0, 1, 2}`** (gates cf-view's tab10 categorical heuristic), exact per-shell
populations (inner **1344** / middle **1800** / outer **3624**, partitioning
all **6768** tets), and the **`interface_flags`-all-false contract** — this
module never calls `with_interface_sdf`, the exact complement of `blended`'s
positive flagging. Emits the categorical `material_layer_id` scalar.

### `blended` — `BlendedScalarField` (smooth cubic-Hermite smoothstep)
The same body sphere + bbox as `layered`, but a smoothstep transition between
two Lamé regions (`μ = 5e4→2e5`) across a band `‖p‖ ∈ [0.055, 0.085]`.
Validates the blended `(μ, λ)` against a test-side smoothstep (`s²(3−2s)`) +
FMA re-derivation at `EXACT_TOL = 0.0`, **bit-exact `s=0`/`s=1` snap** outside
the band (`.to_bits()` equal to pure inner/outer NH), a **monotone
non-decreasing μ gradient** across the band, and the **positive per-tet
`interface_flags` book rule** (`|φ(x_c)| < L_e`, IV-6 — HEADLINE 2, the
complement of `layered`). Exact bucket populations (inside-snap **1056** / band
**2736** / outside-snap **2976**; **3480** interface-flagged — a distinct band).
Emits three scalars: `interface_flag`, `material_mu`, `smoothstep_weight`.

The two are complementary, not subsuming: `blended`'s monotone-gradient +
positive-flag oracles exercise `BlendedScalarField::sample` + `with_interface_sdf`
paths `layered` cannot reach; `layered` is the sole coverage of the
no-interface-SDF partition path + categorical cardinality.

## Run

```
cargo run -p example-scalar-field-stress-test --release
```

Expected: each module prints its anchor-group summary (`Anchor groups (all
assertions exit-0 on success)`) and the binary exits 0 — a clean exit-0 IS the
correctness signal (there is no `PASS` token; a failed assert aborts with 101).

## Visual artifacts

Each module writes a per-tet centroid PLY (a thin `|z| < cell_size/2` z-slab,
648 of 6768 centroids — projecting the shell structure onto a legible disk) to
`out/` for the `cf-viewer` visual-review path (per-vertex scalars auto-detected
and colormapped):

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
