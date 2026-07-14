# stretch — Stress Test (sim-soft uniaxial-stretch validation superset)

Headless validation of the sim-soft uniaxial-stretch surface on the canonical
compressible Neo-Hookean baseline (`μ = 1e5`, `Λ = 4e5` Pa, `ν ≈ 0.4`) — a
deliberate **solver → constitutive → assembly** ladder. No window, no Bevy —
self-gating assertions that abort (exit 101) on any mismatch, so `cargo xtask
run-validators` runs it red-or-green.

Folded from three former per-concept examples (`single-tet-stretch` row 4,
`neo-hookean-uniaxial` row 5, `multi-element-stretch` row 6), each now a module
preserving its hand-authored fixture and oracle checks verbatim. One domain →
one stress-test. The domain straddles the inventory's Tier 1 (row 4) and Tier 2
(rows 5–6); the three modules are complementary, not subsuming — each is the
sole coverage of one distinct axis.

## Modules

### `single_tet` (row 4) — walking-skeleton `Solver::step`
`SkeletonSolver::step` (backward-Euler Newton with `NeoHookean`, no contact) on
`SoftScene::one_tet_cube` — the canonical decimeter-edge tet with `θ = 10 N`
along `+ẑ` on `v_3` and `v_0..v_2` Dirichlet-pinned. Converges in **3 Newton
iters** (exact-pinned) to `dz ≈ 9.692e-4 m` (~1 % strain, inside the
dimensional-analysis band `[5e-4, 1.5e-3] m`), with the pinned DOFs held to
`1e-14 m` and off-axis `v_3.x`/`v_3.y` below `1e-5 m`. All **12 DOFs of
`x_final` match the IV-1 frozen-reference bit pattern** (captured at sim-soft
`c3729d4a`, rustc 1.95.0). Sole coverage of the single-element end-to-end
`Solver::step` path. JSON-only (`out/single_tet/force_stretch.json`).

### `neo_hookean` (row 5) — direct-eval constitutive vs closed form
Direct evaluation of `NeoHookean::first_piola` / `NeoHookean::energy` against the
closed-form NH stress and energy for **traction-free uniaxial stretch**
(`F = diag(λ, λ_t, λ_t)`, `λ_t` from a 1-D inner Newton on the traction-free
transcendental). A **12-point sweep** across `λ ∈ [0.15, 1.95]` (asymmetric
density; in-domain bracket `λ ≈ [0.1182, 2.0]`) validates analytic `P_11`,
`P_22`, ψ vs observed at rel-tol `1e-12`, a `ValidityDomain` declaration check,
`P_11` monotonicity + sign, rest-config `to_bits == 0`, and **48 captured-bit
self-pins** (`P_11`, `P_22`, ψ, `λ_t` × 12) under the IV-1 dense bit-equal tier.
No solver — the constitutive law is exercised in isolation; sole coverage of the
direct-eval axis. JSON (`out/neo_hookean/force_stretch.json`) + optional
`uv run plot.py` matplotlib 2×2 panel (`out/neo_hookean/force_stretch.png`).

### `multi_element` (row 6) — Phase 2 multi-element FEM assembly
`replay_step` on a 27-vertex / 48-tet hex grid
(`HandBuiltTetMesh::uniform_block(2, 0.1, …)`) under uniform Dirichlet stretch:
all 26 boundary vertices pinned to `D · X_rest` with `D = diag(λ, 1, 1)`,
`λ = 1.20`, and the **single interior vertex (ID 13) left free** starting at
rest so Newton has real work (3 iters, residual `~1e-14 N`). Quasi-static via
`cfg.density = 0` (suppress inertia → pure elasticity equilibrium `x = D · X`).
Per-tet `F` evaluates to `diag(λ, 1, 1)` for **every one of the 48 tets**
(uniformity spread `0.0` Pa on the capture platform), and `P_11 ≈ 9.744e4`,
`P_22 = P_33 ≈ 7.293e4` Pa (`Λ ln λ`, non-zero — constrained transverse), ψ match
closed form at rel-tol `1e-12`. **10 sparse-tier captured-bit self-pins**
(interior `x_final` + representative isolated tet's `F`/`P`/ψ, rel-tol `1e-12`
per the IV-1 sparse-tier contract, 81-DOF faer Cholesky). Sole coverage of the
multi-element assembly path (local → global stitch, interior free DOF, per-tet
uniformity oracle). JSON-only (`out/multi_element/multi_element_stretch.json`).

## Run

```
cargo run -p example-stretch-stress-test --release
```

Expected: each module prints its anchor-group summary (`Anchor groups (all
assertions exit-0 on success)`) and the binary exits 0 — a clean exit-0 IS the
correctness signal (there is no `PASS` token; a failed assert aborts with 101).
Use `--release`: the FEM Newton solves are ~30× slower in debug. The solves
here are small (a single tet and a 48-tet block), so the trio is cheap in
release.

## Artifacts

Each module writes its inspection JSON to a namespaced `out/<module>/`
directory (avoiding the `force_stretch.json` name collision between `single_tet`
and `neo_hookean`). These are force-stretch / per-tet traces, not spatial PLYs —
single-tet topology and homogeneous-deformation uniformity are fully described
by the numeric records, so no `cf-view` render is meaningful (inventory Q4). The
`neo_hookean` sweep additionally renders to a matplotlib panel:

```
uv run examples/sim-soft/stretch/stress-test/plot.py   # reads out/neo_hookean/force_stretch.json → force_stretch.png
```

## Cross-references

- **Inventory**: `sim/L0/soft/EXAMPLE_INVENTORY.md` Tier 1 (row 4) + Tier 2
  (rows 5–6).
- **Constitutive law**: `sim/L0/soft/src/material/neo_hookean.rs` (`NeoHookean`,
  `Material::first_piola` / `energy`, `ValidityDomain`).
- **Solver**: `sim/L0/soft/src/solver/backward_euler/mod.rs` (`CpuNewtonSolver`,
  `SkeletonSolver`, multi-element assembly); `SoftScene::one_tet_cube` +
  `HandBuiltTetMesh::uniform_block` fixtures.
- **IV-1 bit-equal contract**: `sim/L0/soft/tests/invariant_iv_1_uniform_passthrough.rs`
  (the two-tier dense-vs-sparse contract + failure-mode protocol the bit-pins
  cite).
- **Book**: Part 2 Ch 04 (hyperelastic Neo-Hookean energy + tangent).
