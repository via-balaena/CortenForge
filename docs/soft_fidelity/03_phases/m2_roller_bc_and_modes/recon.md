# M2 — Roller/per-axis BCs + multi-material breadth + mode recon — RECON

*Active recon, opened 2026-06-10. Head-engineer-owned slicing; M1 (silicone uniaxial
measured accuracy) merged to main 2026-06-10 (PR #295, squash `c98da508`). User confirmed
the M2 direction 2026-06-10: **(d)+(a), recon (b)** — roller/per-axis Dirichlet BCs as the
spine, fold in Ecoflex 00-10, and recon a multi-mode oracle before committing to modes.*

> **Generalize the measured-fidelity machinery M1 built. Spine = per-axis (roller) Dirichlet
> BCs — the only M2 option that needs no new oracle, directly unblocks the Layer-2 keystone
> (soft↔rigid coupling), and upgrades M1's own validation from a constant-strain patch test
> to a genuine free-lateral traction coupon that *independently re-discovers* λ_t and
> converges to the measured curve. Then prove the machinery isn't overfit to one material by
> validating Ecoflex 00-10 (the only other real curve in our oracle) through the new coupon.
> In parallel, recon whether a credible published silicone biaxial/planar-shear dataset
> exists at M1's quality bar — that recon decides whether deformation MODES become M2's deep
> arc or defer to M3.**

---

## 0. Why this sequencing (the decision, recorded)

The five candidate axes (from the M1 named deferreds + the natural next axes) are **not
parallel** — they split sharply by *oracle-acquisition risk* and stack in a dependency
order. Two findings from the M1 close-out and a code read fix the decision:

- **The M1 oracle (Marechal et al. 2021) is uniaxial-tension ONLY** (ASTM D412 dogbone) and,
  in the Zenodo v0.0 snapshot, only **Ecoflex 00-10 and 00-30** carry real curves (Dragon
  Skin files are 1-byte Git-LFS placeholders). So:
  - **(a) more silicones** within our existing oracle = exactly **one** material left:
    **Ecoflex 00-10**. Anything beyond needs a brand-new measured source.
  - **(b) more modes** (shear/biaxial) needs a **completely different published oracle** —
    Marechal cannot serve it. Whether one exists at M1's quality bar is an open recon risk.
  - **(c) human soft tissue** needs a new, messier/anisotropic oracle.
  - **(d) roller BCs** and **(e) near-incompressibility** need **no oracle** — they reuse
    the Marechal uniaxial data already vendored.
- **The solver's free-DOF machinery is already DOF-granular.** `full_to_free_idx` is a
  `vec![None; n_dof]` indexed per-DOF (`backward_euler.rs:501`); the symbolic-pattern build,
  lumped mass, and per-iter assembly all read it per-DOF. The *only* vertex-granular spot is
  the free-DOF construction loop (`backward_euler.rs:491–499`), which pushes all three DOFs
  of each unpinned vertex. So per-axis (roller) Dirichlet is a **localized representation +
  construction change**, not a solver rewrite.

**Dependency structure:**

```
(d) roller/per-axis Dirichlet BCs   ← oracle-free, keystone-aligned, PREREQUISITE
     ├── closes M1's deferred S2 gap (true free-lateral coupon, re-discovers λ_t)
     ├── unblocks the Layer-2 keystone (soft↔rigid coupling interface)
     └── prerequisite for ANY new deformation-mode coupon (b)
(a) Ecoflex 00-10   ← cheap breadth, same oracle, proves machinery isn't overfit to 00-30
(b) modes (shear/biaxial)  ← highest fidelity CEILING, but GATED on a new oracle + needs (d)
(e) near-incompressibility ← rides along as a measured TRIGGER if a mode surfaces vol. error
(c) human tissue   ← mission's ultimate Layer-1 target, but premature until modes validated → M3
```

The single thing M1 did **not** prove is that the model is right in any mode *other than the
one it was fit to* — single-mode hyperelastic calibration mispredicting biaxial/shear is the
canonical failure of this exact workflow, so **(b) is the real fidelity payload** of the arc.
But (b) cannot be committed blind: it has an unresolved oracle dependency *and* cannot run
faithfully without (d). Hence: nail (d), pay off fidelity with (a) through the new coupon,
and de-risk (b) with a cheap oracle-recon spike *before* weeks are spent.

---

## 1. Empirical baseline (what's true in the code today)

- **BCs:** `readout::scene::BoundaryConditions { pinned_vertices: Vec<VertexId>,
  loaded_vertices: Vec<(VertexId, LoadAxis)> }` (`scene.rs:846`). `pinned_vertices` is
  **full 3-DOF Dirichlet only** — "all three xyz DOFs constrained." `LoadAxis ∈ {AxisZ,
  FullVector}`. There is **no per-axis / roller constraint** anywhere.
- **Solver:** `CpuNewtonSolver::new` (`backward_euler.rs:340`) builds the free-DOF map from
  an `effective_pinned: BTreeSet<VertexId>` (user pins ∪ orphan-auto-pin). Construction loop
  (491–499) pushes `{3v, 3v+1, 3v+2}` per unpinned vertex; **`full_to_free_idx` (501) is
  already per-DOF**; symbolic pattern (512–533), mass (473–484), and assembly all key off
  `full_to_free_idx` per-DOF. Validation contracts: pinned/loaded in range, **no
  vertex in both pinned and loaded** (load on a clamped DOF is unphysical), loaded vertices
  must be tet-referenced, orphans auto-pinned.
- **M1 coupon (`tests/uniaxial_fem_coupon.rs`):** a **constant-strain patch test** — imposes
  the full analytical `F=diag(λ,λ_t,λ_t)` as boundary Dirichlet (all boundary DOFs pinned to
  the affine field), checks interior nodes are driven back onto affine. It does **not** let
  the lateral faces contract freely — that needs roller BCs, which don't exist. So the coupon
  proves "solver adds no error to a *prescribed* homogeneous state," not "solver
  *independently recovers* λ_t under axial traction." (Documented S2 limitation.)
- **Analytical reference (`material/uniaxial.rs`):** `free_transverse_uniaxial<M>` (root-find
  λ_t s.t. lateral P₁₁=0 on the real `first_piola`) + `fit_yeoh_uniaxial`. This is the
  locking-free reference the free-lateral FEM coupon must reproduce.
- **Measured calibration:** `silicone_table::ECOFLEX_00_30_MEASURED` (Path-3 const,
  `ConstructionSource::Measured`, μ=16679 Pa, C₂=239 Pa; **not** in `ECOFLEX_FAMILY`).
  Gate `tests/uniaxial_measured_accuracy.rs`: datasheet ~85% RMS → measured ~5.8% RMS over
  λ≤2. Vendored oracle `tests/assets/marechal_2021/ecoflex_00_30_uniaxial.json`.

## 2. Thesis

Per-axis (roller) Dirichlet BCs are the highest-leverage next move because they sit at the
intersection of three otherwise-separate needs: (1) they let us build a **symmetry-cell
coupon** — roller-constrain three mutually-orthogonal min-faces (kill rigid-body modes
without over-constraining), traction the +axis face, and let the lateral faces contract
freely — so the FEM **independently re-discovers** the lateral stretch λ_t and the axial
Cauchy stress, closing M1's S2 patch-test gap with a real traction solve; (2) the same
per-axis constraint primitive is what the **Layer-2 keystone** (differentiable soft↔rigid
coupling) needs at the interface, where a rigid body constrains some interface DOFs but not
others; (3) symmetry/roller BCs are the standard way to run **shear and biaxial coupons**, so
(d) is the structural prerequisite for the eventual modes arc (b). None of this needs a new
oracle — it re-validates against the Marechal uniaxial curve we already vendored, now via a
genuine free-lateral solve. We then prove the machinery generalizes across materials with
**Ecoflex 00-10** (the one other real curve), at near-zero marginal cost.

## 3. End-state (M2 done)

A committed, CI-runnable result set:
1. **Per-axis Dirichlet BCs** in `sim-soft`: a roller constraint primitive expressed in
   `BoundaryConditions` (per-vertex per-axis mask), threaded through `CpuNewtonSolver::new`'s
   free-DOF construction + validation contracts, with the existing full-pin call sites
   unchanged (full pin = all-axes mask).
2. A **free-lateral traction uniaxial coupon** (`tests/`) that, under axial traction + roller
   symmetry BCs, independently recovers λ_t and σ_true matching `free_transverse_uniaxial`
   and the measured Marechal curve — the free-lateral solve the kept patch test stood in for.
3. **Ecoflex 00-10** validated through the same machinery: vendored oracle asset, window fit,
   measured-accuracy gate, Path-3 `ECOFLEX_00_10_MEASURED` const — proving non-overfit.
4. A **mode-oracle recon memo** (in this folder) recording whether a credible published
   silicone biaxial/planar-shear dataset exists, with the decision: (b) → M2 deep arc, or
   defer to M3.

## 4. Gap table

| # | Have | Need | Leaf |
|---|---|---|---|
| G1 | Full-3-DOF-pin BCs only | A **per-axis (roller) Dirichlet** primitive in `BoundaryConditions` + solver free-DOF construction | S1 |
| G2 | Constant-strain patch test (prescribed F) | A **free-lateral traction coupon** that re-discovers λ_t under axial load (roller symmetry cell) | S0/S2 |
| G3 | Measured fit for Ecoflex 00-30 only | **Ecoflex 00-10** vendored oracle + window fit + gate + Path-3 const | S3 |
| G4 | Marechal uniaxial oracle only | A **mode-oracle recon** (does published silicone biaxial/shear data exist at our bar?) | S4 (recon) |
| G5 | ν=0.40 retired for *tension* | A **measured trigger** for near-incompressibility (e) if a future mode surfaces vol. error | (b)/M3 |

## 5. Decisions (head-engineer; revisit if S0 data says otherwise)

- **D1 — Spine = per-axis (roller) Dirichlet BCs (option d).** Confirmed by the user
  2026-06-10. Oracle-free, keystone-aligned, prerequisite for modes, and it upgrades M1's own
  coupon. Lead with it.
- **D2 — Roller representation = per-vertex per-axis boolean mask.** Add a roller list to
  `BoundaryConditions` (e.g. `Vec<(VertexId, [bool;3])>` or an axis-mask newtype). Full pin =
  `[true,true,true]` — keep `pinned_vertices` as the ergonomic full-pin shape (back-compat,
  zero churn at existing call sites) and add rollers as a *separate* field, rather than
  rewriting every call site to a mask. The construction loop pushes only the *unconstrained*
  DOFs of each vertex; `full_to_free_idx` is already per-DOF so the pattern/mass/assembly
  propagate untouched. *(Final shape pinned in S1 after a quick spike of both forms.)*
- **D3 — Validation contracts go DOF-aware.** The "no vertex in both pinned and loaded" and
  orphan-auto-pin checks must become *per-DOF*: a vertex may be roller-constrained on one
  axis and loaded on another (legitimate), but a *fully* pinned DOF still can't also be
  loaded on that same DOF. Get this contract exactly right — it is the subtle correctness
  surface.
- **D4 — Coupon = symmetry-cell, not whole-specimen.** Roller the three min-faces (x-min:
  u_x=0, y-min: u_y=0, z-min: u_z=0) to remove the 6 rigid-body modes with *minimal*
  constraint, traction the +x face, leave +y/+z free. This is the textbook 1/8-symmetry
  hyperelastic coupon. Over-constraining (e.g. full-pinning a face) reintroduces the locking
  the patch test masked — exactly R5 from M1.
- **D5 — Ecoflex 00-10 reuses the M1 machinery verbatim.** Same asset shape, same
  `fit_yeoh_uniaxial`, same gate harness, same Path-3 const pattern (standalone const, NOT
  in `ECOFLEX_FAMILY` — the M1 single-provenance-family gotcha holds). Window/tol set by its
  own S0-style mini-check (likely λ≤2, same as 00-30; confirm — 00-10 is softer).
- **D6 — Modes (b) gated on the S4 oracle recon.** Do NOT write any biaxial/shear code until
  S4 confirms a credible published oracle. If none exists at our bar, (b) defers to M3 and
  M2 closes on (d)+(a). No speculative mode machinery.
- **D7 — Near-incompressibility (e) stays a measured trigger.** Retired for tension in M1;
  revisit only if a *measured* mode (biaxial/compression/contact) shows ν=0.40 caps accuracy
  — surfaced as a number, same discipline as M1's R1. Not M2 scope unless (b) lands and bites.

## 6. Sub-leaf ladder

- **S0 — roller-coupon spike (THROWAWAY, `#[ignore]`, not committed).** The #1-risk de-risk.
  Hand-wire a per-DOF free-DOF map on a small box of Tet4 (single element or a 6-tet cube),
  roller the three min-faces, traction the +x face for the measured-fit Ecoflex 00-30 Yeoh,
  solve one static step, and check: (i) the Hessian is non-singular (constraint count removes
  exactly the 6 rigid-body modes, no more); (ii) the recovered axial stretch + **lateral
  stretch λ_t** match `free_transverse_uniaxial` to mesh tolerance; (iii) σ_true matches the
  measured curve. **Answers before any production commit:** does the symmetry-cell converge,
  does the FEM *re-discover* λ_t (not just reproduce a prescribed one), and is the
  per-DOF-map change as localized as the code read suggests. *(De-risk first, like the M1/MSK
  spikes.)*
- **S1 — per-axis Dirichlet BC primitive (committed).** Add the roller field to
  `BoundaryConditions` (D2), thread it through `CpuNewtonSolver::new` free-DOF construction +
  DOF-aware validation contracts (D3). Unit tests: a roller-on-one-axis vertex frees the
  other two DOFs (free-DOF count is exact); full-pin call sites unchanged (bit-identical
  free-DOF map); the validation contracts reject the right misconfigurations. Keep the public
  surface minimal and keystone-shaped.
- **S2 — free-lateral traction coupon (the free-lateral solve the kept patch test stood in
  for).** Add the symmetry-cell coupon (D4) alongside `uniaxial_fem_coupon.rs`: axial drive, roller min-
  faces, free lateral. Gate: FEM independently recovers λ_t and σ_true ≡ `free_transverse_
  uniaxial` (solver fidelity) AND matches the measured Marechal curve over λ≤2 (model
  fidelity), now without any prescribed lateral stretch. This is the real S2 the M1 patch
  test stood in for.
- **S3 — Ecoflex 00-10 measured fidelity (breadth).** Vendor the 00-10 oracle asset (same
  curation recipe, regenerate from Zenodo `3611329`), a mini window/tol check, the
  measured-accuracy gate, and a Path-3 `ECOFLEX_00_10_MEASURED` const. Proves the machinery
  generalizes across materials. Bank the 00-10 datasheet-vs-measured gap.
- **S4 — mode-oracle recon (memo, may be a small spike).** Search for a credible published
  silicone biaxial / planar-shear stress–stretch dataset (raw curves + provenance) at M1's
  bar. Record the finding + the (b)→M2-deep-arc-or-M3 decision in this folder. *(Can run in
  parallel with S1–S3; it gates the NEXT arc, not these leaves.)*

Each leaf: n+1 cold-read + pre-PR local ultra-review ([[feedback-pre-pr-local-ultra-review]]);
no push / PR without user go-ahead. Slicing is head-engineer-owned
([[feedback-head-engineer-owns-technical-calls]]).

## 7. Risks

- **R1 (#1) — per-axis free-DOF map correctness / rigid-body-mode count.** A symmetry-cell
  coupon needs *exactly* enough roller constraint to remove the 6 rigid-body modes; too few →
  singular Hessian (faer Cholesky panics/garbage), too many → reintroduces volumetric locking
  that masquerades as model error (this is M1's R5 resurfacing in the BC layer). **MITIGATION:
  S0 spike builds the per-DOF map by hand and checks non-singularity + λ_t recovery before any
  production code.** The code read shows `full_to_free_idx` is already per-DOF, so the change
  is localized — the risk is the *constraint design*, not the solver plumbing.
- **R2 — BC representation churn / back-compat.** Adding rollers must not break the dozens of
  full-pin call sites (`scene.rs`, every existing test). MITIGATION: D2 keeps `pinned_vertices`
  as-is and adds rollers as a separate field; full-pin free-DOF map stays bit-identical.
- **R3 — DOF-aware validation contract subtlety.** A roller vertex may be loaded on a *free*
  axis but not a *constrained* one; the no-overlap check must go per-DOF without rejecting
  legitimate roller+load combos or admitting unphysical clamped-DOF loads. MITIGATION: D3 +
  explicit unit tests for each contract case.
- **R4 — Ecoflex 00-10 data quality.** The 00-10 curve may have more toe-region noise (it's
  the softest grade) or a different usable window. MITIGATION: S3 mini window/tol check mirrors
  M1's S0; don't assume λ≤2 transfers.
- **R5 — mode oracle may not exist at our bar.** (b) could be un-de-riskable from public data.
  MITIGATION: S4 is explicitly a *recon* that can conclude "defer to M3"; M2's committed value
  ((d)+(a)) does not depend on it.

## Progress · S0 — DONE (2026-06-10, throwaway spike, deleted when S1 landed)

R1 (#1 risk) RETIRED. A churn-free standalone mini-FEM (`tests/spike_roller_bc_s0.rs`,
`#[ignore]`) built a 1/8-symmetry cube cell (8 nodes / 6 Tet4) with 3 min-face rollers +
prescribed axial faces + free lateral, hand-rolled per-DOF free-DOF condensation + Newton (FD
tangent, plus a built-in `G=−V·P·Dm⁻ᵀ ≡ −dE/dx` force-assembly self-check). Across
λ∈{1.1,1.25,1.5,1.75,2.0}, on BOTH NeoHookean and the shipped `ECOFLEX_00_30_MEASURED` Yeoh:
converges in 4–6 Newton iters to ~1e-13; **independently re-discovers λ_t to ~1e-14** vs
`free_transverse_uniaxial` (it was *solved*, not prescribed — the thing S2's patch test could
not do); axial Cauchy matches ~1e-13 rel; recovered `F` homogeneous-diagonal (F₂₂≈F₃₃,
off+aniso ~1e-16). ⇒ the per-DOF roller symmetry-cell is well-posed (removes exactly the 6
rigid-body modes) and re-discovers λ_t. Combined with the code read (the solver's
`full_to_free_idx` is already per-DOF), both halves of the (d) risk are closed. Spike deleted
when S1 landed (de-risk banked here + in memory).

## Progress · S1 — SHIPPED (2026-06-10, branch `feat/soft-fidelity-m2-roller-bc`, NOT pushed)

Production **roller / per-axis Dirichlet BC primitive**:
- `BoundaryConditions` gains `roller_vertices: Vec<(VertexId, [bool; 3])>` (mask `[x,y,z]`,
  `true` = that DOF held at its initial `x_prev` value — rest for a symmetry plane, a
  prescribed offset for a displacement-driven face) + a `const fn new(pinned, loaded)` constructor
  (empty rollers) (`readout/scene.rs`). `pinned_vertices` unchanged → full-pin scenes stay
  back-compatible.
- `CpuNewtonSolver::new` (`solver/backward_euler.rs`): builds a dense `roller_mask` +
  DOF-aware validation (roller IDs in range; no all-`false` mask; no duplicate roller vertex;
  no vertex in both `pinned_vertices` and `roller_vertices`; no vertex in both
  `loaded_vertices` and `roller_vertices` — a loaded vertex must be free on all three axes, as
  the assembly + autograd VJP resolve all xyz free indices). The free-DOF construction loop is
  now **per-DOF**: a full pin (user ∪ orphan) fixes all three axes, a roller fixes only its
  `true` axes. A scene with no rollers pushes 3v,3v+1,3v+2 in order ⇒ a **bit-identical**
  free-DOF map to the pre-roller construction (regression-tested). The symbolic pattern, lumped
  mass, and per-iter assembly already read `full_to_free_idx` per-DOF, so they propagate
  untouched.
- **10 unit tests** (solver test module): roller frees exactly the unconstrained axes;
  no-roller map unchanged (= [9,10,11] for the 1-tet scene); all-`true` roller ≡ full pin;
  constructor leaves rollers empty; a **driven roller holds its DOF at a nonzero `x_prev`
  offset** through a solve (the displacement-driven mechanism); + 5 `#[should_panic]`
  validation contracts.
- All BC construction sites updated (34 pre-existing in sim-soft lib/tests + 12 in
  examples/tools; the new roller unit tests add their own).
  sim-soft suite green (44 binaries); **workspace `build --all-targets` clean**; grade **A**
  (all automated tiers; manual API-design tier `?` as expected).

## Progress · S2 — SHIPPED (2026-06-10, same branch, NOT pushed)

The genuine **roller free-lateral uniaxial coupon** (`tests/uniaxial_roller_coupon.rs`), run
through the real `CpuTet4YeohSolver` on the **measured** material (`ECOFLEX_00_30_MEASURED`):
- A test-local `YeohBox` implements `Mesh<Yeoh>` by **delegating** topology + the diagnostic
  surfaces (adjacency/quality/interface_flags/boundary_faces) to an inner `HandBuiltTetMesh`
  (the validated box generator) and overriding only `materials()` to carry the measured Yeoh.
  The solver reads only the 5 topology+material methods, so the delegated aux is never touched
  on the hot path — zero production-mesh churn (no generic refactor of `HandBuiltTetMesh`).
- A `1/8`-symmetry cell: rollers on the three min faces (`x=0`/`y=0`/`z=0`, removing exactly
  the 6 rigid-body modes), `x=L` driven to `(λ−1)L`, **`+y`/`+z` faces free**. Free DOFs
  warm-started from a **neutral** guess (lateral at REST, `λ_t=1` — the axial is prescribed by
  the driven faces, not the claim), then perturbed; the lateral contraction is **discovered
  from a neutral basin, not seeded** (an ultra-review hardening over the initial affine seed).
- **Two gates, both green:** (1) solver fidelity — across λ∈{1.1,1.25,1.5,1.75,1.9} the FEM
  **re-discovers λ_t** (a direct read-off from a free `+y`-face node matches
  `free_transverse_uniaxial` to <1e-6) + recovered `F ≡ diag(λ,λ_t,λ_t)` + axial Cauchy ≡
  analytical (<1e-6 rel) + lateral traction ~0; (2) model fidelity — the measured Yeoh
  reproduces Marechal's curve at **5.8 % RMS over λ≤2** (M1 gate re-asserted on the asset).
  Chained: the coupon reproduces *measurement* to ~5.8 %. Solver-driven λ capped at 1.9
  (deviation 0.9 < the Yeoh validity ceiling 1.0); RMS is analytical over full λ≤2.
- The M1 constant-strain patch test (`uniaxial_fem_coupon.rs`) is **kept** (distinct
  multi-element constant-strain verification on the NH path) with its header truthed-up (the
  free-lateral re-discovery it deferred to M2 now lives in the roller coupon). grade A; suite
  green (44 binaries).

**Next: S1+S2 = PR-A** — n+1 cold-read + pre-PR local ultra-review, then propose the PR.
Then S3 (Ecoflex 00-10) and S4 (mode-oracle recon).

## 8. M2 validation gate (definition of done)

CI-runnable, network-free: (1) roller/per-axis Dirichlet BCs expressible and threaded through
the solver, full-pin map bit-identical, DOF-aware contracts tested; (2) the free-lateral
traction coupon independently recovers λ_t and σ_true ≡ analytical and ≡ measured over λ≤2,
(the kept patch test stood in for it); (3) Ecoflex 00-10 reproduces its measured curve within tol over
its window, with a Path-3 const + banked gap; (4) the mode-oracle recon memo records the (b)
decision. Honest scope: still validated against a *published measurement* (now two materials,
genuine free-lateral solve), not our own cast — the hardware tensile gate remains M3+.

## Key files / pointers

- BCs: `sim/L0/soft/src/readout/scene.rs` (`BoundaryConditions`, `LoadAxis`,
  `pick_vertices_by_predicate`); solver `sim/L0/soft/src/solver/backward_euler.rs`
  (free-DOF construction 486–504, `full_to_free_idx`, validation 356–438).
- Reuse: `material/uniaxial.rs` (`free_transverse_uniaxial`, `fit_yeoh_uniaxial`);
  `tests/uniaxial_fem_coupon.rs` (M1 patch test, kept; the roller coupon is its free-lateral
  sibling); `tests/uniaxial_measured_
  accuracy.rs` (gate harness); `tests/invariant_iv_1_uniform_passthrough.rs`.
- Oracle: Marechal et al. 2021, Soft Robotics 8(3):284–297; data Zenodo `10.5281/zenodo.3611329`
  (Ecoflex 00-10 + 00-30 only have real curves; regen recipe in
  `tests/assets/marechal_2021/PROVENANCE.md`).
- Path-3 precedent: `silicone_table::ECOFLEX_00_30_MEASURED` (standalone const, NOT in
  `ECOFLEX_FAMILY` — single-provenance-family gotcha).
- M1 recon: `docs/soft_fidelity/03_phases/m1_silicone_uniaxial/recon.md`.
