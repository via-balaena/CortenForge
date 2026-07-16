# material — Demo (sim-soft silicone-material reference table)

Headless **demonstration** of the sim-soft material-reference surface: the
production silicone constants table, with each entry's compressible
Neo-Hookean stress + energy at the σ_100 anchor shown alongside its Smooth-On
data-sheet value. No window, no Bevy, and no self-gating asserts — the printed
table (and `out/silicone_materials.json`) IS the artifact. The underlying
constitutive correctness is validated in the library (see "Where the
correctness lives"), so this example's job is to SHOW the lookup, not check it.

The `material` domain currently has a single capability example
(`silicone-material-table`, row 19), relocated into the standardization layout
as the module `material_table`. The dispatcher shape matches the sibling
`<domain>/stress-test/` folds (sdf / scalar-field / stretch / bonded), so a
second material example would slot in as another `mod` + `run()` call with no
restructuring.

## Modules

### `material_table` (row 19) — silicone constants + compressible-NH stress (demo)
Direct-eval consumer of F4's
[`sim_soft::material::silicone_table`](../../../../sim/L0/soft/src/material/silicone_table.rs):
iterates the seven `pub const SiliconeMaterial` entries (`{ECOFLEX_00_10 / _20 /
_30 / _50, DRAGON_SKIN_10A / _20A / _30A}`), dispatches each via
`SiliconeMaterial::to_neo_hookean()` (`const` bridge into the `Material` trait),
and probes the resulting `NeoHookean` at `F = diag(2.0, 1, 1)` — simple uniaxial
stretch at `λ = 2.0`, the Smooth-On `σ_100 = 100 % engineering strain`
data-sheet anchor (`ε = 1.0 ⇒ λ = 2`). **No solver, no mesh** — the constitutive
table is the artifact. Output is `out/silicone_materials.json` (a
programmatic-consumption lookup; JSON-only per inventory Q4 row 19 — the table
IS the artifact, cf-view is irrelevant for material-reference data).

Companion to row 5 (the `neo_hookean` module of
[`stretch/stress-test`](../../stretch/stress-test)): row 5 sweeps one material
across `λ ∈ [0.15, 1.95]` under traction-free uniaxial (transcendental `λ_t`);
row 19 sweeps seven materials at one fixed `F` under simple stretch (closed-form
scalar, no inner Newton). Together they cover the constitutive story from two
angles — row 5 (a validator) checks `Material::first_piola` + `energy` across
the in-domain bracket on one material; row 19 (this demo) exhibits the
production library's `to_neo_hookean()` const bridge on all seven at the
data-sheet anchor. Row 19 asserts nothing — the bridge's correctness is
covered by `silicone_table.rs`'s and `neo_hookean.rs`'s own lib tests.

**Closed forms** (compressible NH at `F = diag(λ, 1, 1)`, `J = λ`, `Λ = 4μ` ⇒
`ν = 0.40`):

```text
ψ    = (μ/2)(λ² − 1) − μ ln(λ) + (Λ/2)(ln λ)²
P_11 = μ(λ − 1/λ) + Λ ln(λ) / λ        ≈ 2.886 μ  at λ = 2
P_22 = P_33 = Λ ln(λ)                  ≈ 2.773 μ  at λ = 2
```

`P_11 ≈ 2.886 μ` sits ~3.8 % below the `3μ` small-strain identity Smooth-On's
`σ_100` is conventionally interpreted under — the finite-strain compressible-NH
simple-stretch correction at `ν = 0.40`. That constitutive gap plus the
data-sheet's catalog-value uncertainty is exactly what Fork B's post-cast
modulus fit absorbs; the contract is "the table reproduces the data sheet up to
a known finite-strain correction," not "bit-exact to the data sheet."

**Where the correctness lives** (this demo does not self-gate; every property
these readouts illustrate is validated by a lib unit test that covers every
entry in this table):

- **`NeoHookean` closed-form `first_piola` / `energy` value + `P_22 == P_33`
  transverse symmetry + rest-config zero** → `neo_hookean.rs`'s `#[cfg(test)]`
  unit tests (material-parameter-independent, so one test covers all seven).
- **`λ = 4μ` (ν = 0.40) + μ-non-decreasing-along-hardness ordering** →
  `silicone_table.rs`'s own `#[cfg(test)]` tests
  (`lambda_is_four_times_mu_at_nu_0_40` / `mu_is_non_decreasing_along_hardness_order`).

The σ_100 data-sheet values are decorative reference points — printed and
serialized alongside the computed `P_11`, never asserted (the demo's contract
is not "the table is the data sheet bit-exact").

## Run

```
cargo run -p example-material-stress-test --release
```

Expected: the demo prints the 7×material table, writes the JSON, and exits 0.
Release mode is the workspace convention across the sim-soft examples arc (this
row is purely analytic — seven `first_piola` + `energy` evals on a
`Matrix3<f64>`, milliseconds either way — but `--release` keeps the invocation
shape uniform).

## Artifacts

`out/silicone_materials.json` — programmatic-consumption lookup (object-key order
per `serde_json::json!`, alphabetical; schema documented inline at
`src/material_table.rs::save_json`). No cf-view artifact.

## Cross-references

- **Inventory**: `sim/L0/soft/EXAMPLE_INVENTORY.md` Tier 5 (row 19).
- **Material library**: `sim/L0/soft/src/material/silicone_table.rs` (F4 — the
  `SiliconeMaterial` consts + `to_neo_hookean()` const bridge this row consumes).
- **Constitutive law**: `sim/L0/soft/src/material/neo_hookean.rs` (`NeoHookean`,
  `Material::first_piola` / `energy`).
- **Sister sim-soft fold**: `stretch/stress-test`'s `neo_hookean` module (row 5 —
  the single-material traction-free sweep this row complements).
