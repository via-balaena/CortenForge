# stretch — Stress Test (sim-soft uniaxial-stretch validation superset)

Headless validation of the sim-soft uniaxial-stretch surface on the canonical
compressible Neo-Hookean baseline (`μ = 1e5`, `Λ = 4e5` Pa, `ν ≈ 0.4`) — a
deliberate **solver → constitutive → assembly** ladder. No window, no Bevy —
self-gating assertions that abort (exit 101) on any mismatch, so `cargo xtask
run-validators` runs it red-or-green.

Folded from three former per-concept examples (`single-tet-stretch` row 4,
`neo-hookean-uniaxial` row 5, `multi-element-stretch` row 6), each now a module
driving the real engine and emitting an inspectable trace. Per Rule-B, the
`single_tet` and `neo_hookean` modules delegate physical / constitutive
correctness to the `sim-soft` lib tests and self-gate only on
demonstration-integrity; `multi_element` still carries the assembly oracle.
One domain → one stress-test. The domain straddles the inventory's Tier 1 (row 4) and Tier 2
(rows 5–6); the three modules are complementary, not subsuming — each is the
sole coverage of one distinct axis.

## Modules

### `single_tet` (row 4) — walking-skeleton `Solver::step` (demonstration)
`SkeletonSolver::step` (backward-Euler Newton with `NeoHookean`, no contact) on
`SoftScene::one_tet_cube` — the canonical decimeter-edge tet with `θ = 10 N`
along `+ẑ` on `v_3` and `v_0..v_2` Dirichlet-pinned. Converges in 3 Newton iters
to `dz ≈ 9.692e-4 m` (~1 % strain), which the module emits as the DOF trace.
Self-gates only that **the solve converged** (`iter_count < max_newton_iter`,
`residual < tol`), so the emitted trace is a real converged solution. The
scene's physical correctness — `x_final` bit-equality vs the IV-1 reference,
Dirichlet-pin enforcement, `+ẑ` displacement sign, F=I axis alignment, and the
dimensional-analysis band `[5e-4, 1.5e-3] m` — is owned by the `sim-soft` lib
tests `solver_convergence.rs::stage_1_traction_converges` (byte-for-byte this
scene) + `invariant_iv_1_uniform_passthrough.rs` (the `x_final` bit contract),
not re-asserted here. Sole coverage of the single-element end-to-end
`Solver::step` demonstration. JSON-only (`out/single_tet/force_stretch.json`).

### `neo_hookean` (row 5) — traction-free force-stretch curve (demonstration)
Drives the real `NeoHookean` across a **12-point traction-free uniaxial sweep**
(`F = diag(λ, λ_t, λ_t)`, `λ_t` from a 1-D inner Newton on the traction-free
transcendental) over `λ ∈ [0.15, 1.95]` (asymmetric density; in-domain bracket
`λ ≈ [0.1182, 2.0]`) and emits the force-stretch curve as JSON — the artifact.
No solver — the constitutive law is exercised in isolation. Demonstration
self-gates read the **real** `first_piola`/`energy` output: inner-Newton
convergence, traction-free `P_22`/`P_33`, `P_11` monotonicity + sign, and
sweep-in-domain (bound from the live `validity()`).

Constitutive **closed-form correctness** (`first_piola`/`energy` vs analytic at
rel `1e-12`) is owned by the `sim-soft` `NeoHookean` lib tests — `diag(s,1,1)`
uniaxial and `diag(a,b,b)` general-transverse — not re-asserted here. The JSON
and plot carry the **real observed** curve only (no analytic overlay, so nothing
in the artifact is unverified). JSON (`out/neo_hookean/force_stretch.json`) +
optional `uv run plot.py` matplotlib 2×2 panel (`out/neo_hookean/force_stretch.png`).

### `multi_element` (row 6) — Phase 2 multi-element FEM assembly
`replay_step` on a 27-vertex / 48-tet hex grid
(`HandBuiltTetMesh::uniform_block(2, 0.1, …)`) under uniform Dirichlet stretch:
all 26 boundary vertices pinned to `D · X_rest` with `D = diag(λ, 1, 1)`,
`λ = 1.20`, and the **single interior vertex (ID 13) left free** starting at
rest so Newton has real work (3 iters, residual `~1e-14 N`). Quasi-static via
`cfg.density = 0` (suppress inertia → pure elasticity equilibrium `x = D · X`).
A heavy end-to-end FEM solve: per-tet `F` evaluates to `diag(λ, 1, 1)` for
**every one of the 48 tets** with a bounded `P_11` uniformity spread (`0.0` Pa
on the capture platform), plus the interior vertex reproducing `D · X_rest[13]`.
The FEM assembly machinery is lib-owned — `tests/uniaxial_fem_coupon.rs` is a
multi-element (384-tet) **traction-free** (`F = diag(λ, λ_t, λ_t)`) constant-strain
patch test — so this example's distinct axis is the **constrained-transverse**
`diag(λ, 1, 1)` config (`λ_t = 1` prescribed, not the coupon's traction-free
solve) plus the explicit all-48 per-element uniformity readout. The real per-tet
`P`/ψ (`P_11 ≈ 9.744e4`, `P_22 = P_33 ≈ 7.293e4` Pa, `Λ ln λ` — non-zero,
constrained transverse) go to the JSON as observed values; their **closed-form
correctness is owned by the `sim-soft` `NeoHookean` lib tests** (`diag(s,1,1)` +
`diag(a,b,b)`), not re-asserted here. Sole coverage of the multi-element
assembly path (local → global stitch, interior free DOF, per-tet uniformity).
JSON-only (`out/multi_element/multi_element_stretch.json`).

## Run

```
cargo run -p example-stretch-stress-test --release
```

Expected: each module prints its scene + gate summary (the gate header varies
by module — e.g. `multi_element`'s `Anchor groups`, `single_tet`'s
`Demonstration gate`) and the binary exits 0 — a clean exit-0 IS the
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
  (owns `single_tet`'s `x_final` bit-equality — the de-fragged modules delegate
  their bit-level regression net here rather than self-pinning captured bits).
- **Single-tet convergence + scene physics**:
  `sim/L0/soft/tests/solver_convergence.rs` (owns the `single_tet` scene's
  Dirichlet pins, displacement sign / band, F=I axis alignment).
- **Book**: Part 2 Ch 04 (hyperelastic Neo-Hookean energy + tangent).
