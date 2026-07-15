# material — Stress Test (sim-soft silicone-material validation superset)

Headless validation of the sim-soft **material-reference** surface: the
production silicone constants table validated against closed-form compressible
Neo-Hookean stress + energy. No window, no Bevy — self-gating assertions that
abort (exit 101) on any mismatch, so `cargo xtask run-validators` runs it
red-or-green.

The `material` domain currently has a single capability example
(`silicone-material-table`, row 19), relocated into the standardization layout
as the module `material_table`. The dispatcher shape matches the sibling
`<domain>/stress-test/` folds (sdf / scalar-field / stretch / bonded), so a
second material example would slot in as another `mod` + `run()` call with no
restructuring.

## Modules

### `material_table` (row 19) — silicone constants vs closed-form NH
Direct-eval consumer of F4's
[`sim_soft::material::silicone_table`](../../../../sim/L0/soft/src/material/silicone_table.rs):
iterates the seven `pub const SiliconeMaterial` entries (`{ECOFLEX_00_10 / _20 /
_30 / _50, DRAGON_SKIN_10A / _20A / _30A}`), dispatches each via
`SiliconeMaterial::to_neo_hookean()` (`const` bridge into the `Material` trait),
and probes the resulting `NeoHookean` at `F = diag(2.0, 1, 1)` — simple uniaxial
stretch at `λ = 2.0`, the Smooth-On `σ_100 = 100 % engineering strain`
data-sheet anchor (`ε = 1.0 ⇒ λ = 2`). **No solver, no mesh** — the constitutive
table is the artifact. Seven anchor groups (see below); output is
`out/silicone_materials.json` (a programmatic-consumption lookup; JSON-only per
inventory Q4 row 19 — the table IS the artifact, cf-view is irrelevant for
material-reference data).

Companion to row 5 (the `neo_hookean` module of
[`stretch/stress-test`](../../stretch/stress-test)): row 5 sweeps one material
across `λ ∈ [0.15, 1.95]` under traction-free uniaxial (transcendental `λ_t`);
row 19 sweeps seven materials at one fixed `F` under simple stretch (closed-form
scalar, no inner Newton). Together they close the constitutive-coverage story —
row 5 validates `Material::first_piola` + `energy` across the in-domain bracket
on one material; row 19 validates the production library's `to_neo_hookean()`
const bridge on all seven at the data-sheet anchor.

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

**Anchor groups** (all assertions exit-0 on success; `cargo run --release`
exit-0 IS the correctness signal):

1. **`nu_invariant`** — per material, `λ_pa.to_bits() == (4·μ_pa).to_bits()`
   (F4's `lambda_is_four_times_mu_at_nu_0_40` lifted as a cross-crate net).
2. **`rest_config_zero`** — at `F = I`, `P_11` / `P_22` / ψ all `0u64` bit-exact.
3. **`closed_form_p11`** — observed vs `μ(λ−1/λ) + Λ ln(λ)/λ` at rel `1e-12`.
4. **`closed_form_p22`** — observed vs `Λ ln(λ)`; `P_33 == P_22` within `1e-8 Pa`.
5. **`closed_form_psi`** — observed vs closed-form ψ at rel `1e-12`.
6. **`hardness_ordering`** — `P_11` non-decreasing along source-PSI order
   (Ecoflex 00-10 / 00-20 tie permitted at 8 PSI).
7. **`captured_bits`** — per-material `(P_11, P_22, ψ)` exact `to_bits()` self-pin
   (21 pins, IV-1 dense bit-equal tier — pure-analytic probe, no sparse-solver
   FMA path). Failure-mode protocol per IV-1: on a drift, rule out toolchain
   before suspecting a real regression in `Material::first_piola` / `energy` or
   F4's `to_neo_hookean`; NEVER re-bake to silence it. Re-bake helper (on an
   intentional F4 const edit only): `CF_CAPTURE_BITS=1 cargo run -p
   example-material-stress-test --release` prints the block and skips the pin.

## Run

```
cargo run -p example-material-stress-test --release
```

Expected: the module prints its anchor-group summary + the 7×material table and
exits 0. Release mode is the workspace convention across the sim-soft examples
arc (this row is purely analytic — seven `first_piola` + `energy` evals on a
`Matrix3<f64>`, milliseconds in debug — but `--release` keeps the invocation
shape uniform, and the captured bits were captured under release).

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
