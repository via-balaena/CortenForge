# cf-sdf trait-split + sim-soft layer-integrity close — RECON

> **Status**: RECON SHIPPED 2026-05-20. Bookmark + recon + planning
> in this session per the three-session pattern; implementation in
> the next session, intended to land on `dev` and append to PR #249.
>
> **Triggered by**: PR #249's CI layer-integrity failure for sim-soft
> (108 transitive deps > L0 tier max 100). Previous fix `9eb6cd59`
> gated cf-design's `parallel-meshing` feature behind a default-on
> flag and sim-soft opts out → 103 deps, still 3 over. This recon
> covers the remaining gap PLUS the architectural-direction fix the
> dep-cap surfaced.
>
> **Predecessors**:
> - `docs/PR_249_REVIEW.md` §B2 — surfaces the layer-integrity gap
> - Commit `9eb6cd59` — Phase 0 work: rayon gate (5 deps dropped)

---

## TL;DR

Lift the `Sdf` trait out of cf-design into a thin standalone
`cf-sdf` crate. sim-soft swaps from depending on cf-design (the full
design kernel — Solid CSG, adaptive mesher, etc.) to depending on
just the trait contract. Pair with replacing `tracing` usage in
sim-core (4 call sites, dead-weight since no subscriber is
installed anywhere in the workspace) so the layer-integrity count
lands under 100.

Combined effect on sim-soft:
- Original baseline (main): 108 deps.
- After PR #249 commit `9eb6cd59` (rayon gate): 103.
- After this arc (trait-split + tracing→log): **~99**, Layer Integrity A.

---

## 1. Architectural rationale

### 1.1 The smell the dep-cap surfaced

sim-soft is an L0 FEM solver. Per `docs/PROGRAM_GAMEPLAN.md` §7.1
it's in the "research cluster" — foundational, deep stack. Its only
consumption of cf-design is the `Sdf` trait re-export at
`sim/L0/soft/src/sdf_bridge/sdf.rs:44`:

```rust
pub use cf_design::Sdf;
```

…yet pulling `cf-design = { workspace = true }` brings in the full
design kernel: `Solid` CSG, FieldNode tree, adaptive dual-contouring
mesher, mesh_simplified / mesh_to_tolerance, rayon (now gated),
etc. That's a dep-direction smell: a foundational solver crate
should NOT transitively depend on a design surface that USES it.

### 1.2 The right architecture

Three crates instead of one for the SDF concept:

```
cf-sdf            — the Sdf trait + Box/Arc blanket impls.
                    Deps: nalgebra. Tier: L0. Probably ~50 LOC.
cf-design         — Solid + CSG + adaptive mesher + (kept) impl Sdf for Solid.
                    Re-exports `pub use cf_sdf::Sdf;` for backward compat.
mesh-sdf          — TriMeshDistance + Sign oracles + (moved) impl Sdf for
                    Signed<D, S> + impl Sdf for CachedGridSdf.
```

cf-sdf is the trait contract. cf-design and mesh-sdf each implement
it for their own types. sim-soft depends on the contract, not the
kernel. No more orphan-rule contortion in cf-design's `sdf.rs`.

---

## 2. Recon findings

### 2.1 The `Sdf` trait surface is tiny

`design/cf-design/src/sdf.rs` (351 LOC total):

- Lines 28-34: trait definition (`Sdf: Send + Sync` with `eval` +
  `grad`). ~7 LOC.
- Lines 42-50: blanket `impl<T: Sdf + ?Sized> Sdf for Box<T>`.
- Lines 61-69: blanket `impl<T: Sdf + ?Sized> Sdf for Arc<T>`.

Total cf-sdf lift surface: **~30 LOC of impl** + module docstring.
Deps: nalgebra (`Point3<f64>`, `Vector3<f64>`).

The other 320 LOC in cf-design's `sdf.rs` are:

- `impl Sdf for Solid` (8 LOC) — STAYS in cf-design (needs `Solid`).
- `impl Sdf for Signed<D, S>` (25 LOC) — MOVES to mesh-sdf.
- `impl Sdf for CachedGridSdf` (10 LOC) — MOVES to mesh-sdf.
- Tests (8 fns covering all four impls + the blankets).

### 2.2 cf-design's actual contribution to sim-soft's tree is tiny

**Key recon measurement** (sequential `cargo tree -p $dep -i $x`
over every dep cf-design contributes):

| Dep cf-design contributes | Also reachable via sim-soft's other deps? |
|---|---|
| All 57 of them | **Yes — every one** |

The intuition `cf-design = 62 unique transitive deps → removing it
drops 62` is wrong. cf-design's transitive set is fully overlapped
with sim-soft's other direct deps (mesh-offset → mesh-sdf →
parry3d/nalgebra/etc.; sim-ml-chassis → sim-core → tracing/serde/
etc.; mesh-types → cf-geometry/nalgebra). Removing cf-design from
sim-soft's direct deps would save only `cf-design` itself = **1 dep**.

This **does not close** the 103 → 100 layer-integrity gap on its own.

### 2.3 The trait-split is still the right fix

Even though it saves only 1 dep COUNT-wise, the trait-split:

- Fixes the architectural-direction smell (L0 solver no longer
  depends on the design kernel).
- Reduces sim-soft's compile-time coupling (changes to
  `Solid::user_fn` no longer recompile sim-soft).
- Documents the contract clearly (sim-soft uses the trait, not the
  kernel — the `pub use` line at `sdf_bridge/sdf.rs:44` becomes
  honest).
- Future-proofs sim-soft against cf-design growth (any new heavy
  machinery added to cf-design — say, a TetMesh kernel — doesn't
  leak into sim-soft's tree).

So we ship the trait-split + ONE additional cut to close the
layer-integrity gap.

### 2.4 The additional cut: tracing → log in sim-core

`sim/L0/core/src/` has exactly **4 sites** using `tracing`:

- `types/warning.rs:63` — `tracing::warn!`
- `forward/check.rs:100` — `tracing::error!`
- `forward/mod.rs:177` — `tracing::warn!`
- `collision/mesh_collide.rs:15` — `use tracing::warn;`

Workspace-wide `tracing-subscriber` check: **none installed**.
Tracing output goes to `/dev/null` in production. The dep is
strict dead-weight (3 deps: `tracing`, `tracing-attributes`,
`tracing-core`).

`log` is already in sim-soft's transitive tree (via `parry3d →
ena → log`), so swapping sim-core's tracing for log adds zero
new deps. The migration is mechanical (~6 LOC across 4 sites).

Combined savings: trait-split (1 dep) + tracing→log (3 deps) = 4
deps. sim-soft lands at **103 - 4 = 99** → Layer Integrity A.

### 2.5 cf_design::Sdf consumer enumeration

14 source files use `cf_design::Sdf` directly (re-export or `use`):

- `mesh-sdf/src/{oracle.rs, lib.rs}` — host the adapter impls
  (these MOVE in Phase 2)
- `cf-cast/src/ribbon.rs`, `cf-cast-cli/src/{derive.rs, scan.rs}`,
  `cf-device-geometry/src/sdf_layers.rs`, `cf-design/src/solid_layered.rs`,
  `cf-sim-research/src/insertion_sim.rs` — consume the trait
  generically, will keep working through cf-design's re-export
- 5 examples in `examples/sim-soft/` — same
- `sim/L0/soft/src/sdf_bridge/sdf.rs:44` — the migration target;
  swaps to `cf_sdf::Sdf` directly

Only sim-soft needs an actual source-side change. All other
consumers continue using `cf_design::Sdf` via the re-export.

---

## 3. Migration plan

Six phases. Each phase ends with a green workspace build.

### Phase 1 — Create `cf-sdf` crate skeleton

- New crate at `design/cf-sdf/`. Workspace tier metadata: L0.
- `Cargo.toml`: dep on `nalgebra` (workspace) only. Lenient lint
  posture per [[feedback-lint-posture-on-extract]] — inherit from
  cf-design's posture (the source).
- `src/lib.rs`:
  - Module docstring referencing the lift origin (cf-design's
    `sdf.rs`) and the architectural rationale (sim-soft's L0
    placement vs cf-design's kernel role).
  - `pub trait Sdf: Send + Sync { fn eval; fn grad; }` (verbatim
    from cf-design/src/sdf.rs:28-34).
  - `impl<T: Sdf + ?Sized> Sdf for Box<T>` (verbatim from
    cf-design lines 42-50).
  - `impl<T: Sdf + ?Sized> Sdf for Arc<T>` (verbatim from lines
    61-69).
- `src/lib.rs` tests (lifted from cf-design/src/sdf.rs):
  - `box_dyn_sdf_blanket_forwards_eval_and_grad` (lines 225-236)
  - `arc_dyn_sdf_blanket_forwards_eval_and_grad` (lines 238-255)
  - Adjusted: build a tiny constant-eval `Sdf` impl in the test
    module instead of using `Solid::sphere` (no cf-design dep).
- Workspace `Cargo.toml`: add `design/cf-sdf` to `members`; add
  `cf-sdf = { path = "design/cf-sdf" }` to `[workspace.dependencies]`.
- xtask grader exemption per cf-bevy-common / cf-device-types
  precedent (`xtask/src/grade.rs::applies_to_crate`).
- Verify: `cargo build -p cf-sdf` + `cargo test -p cf-sdf` green;
  2 tests pass.

**Estimate**: ~30 min.

### Phase 2 — mesh-sdf adopts cf-sdf

- `mesh-sdf/Cargo.toml`: add `cf-sdf = { workspace = true }`.
- Move `impl Sdf for Signed<D, S>` (cf-design/src/sdf.rs:113-137)
  → `mesh-sdf/src/sdf_adapter.rs` (or wherever the host module
  fits — recon pass during impl picks the home; mesh-sdf's
  `oracle.rs` or `lib.rs` are candidates).
- Move `impl Sdf for CachedGridSdf` (lines 150-158) → same.
- `mesh-sdf/src/lib.rs`: `pub use cf_sdf::Sdf;` for downstream
  ergonomics (so consumers that use mesh-sdf can `use mesh_sdf::Sdf`
  too; OR they use `cf_sdf::Sdf` directly — both work).
- Tests follow:
  - `mesh_sdf_eval_sign_convention_in_contact_band` (cf-design
    lines 284-298)
  - `mesh_sdf_grad_below_bottom_face_approximates_outward_normal`
    (lines 300-312)
  - `cached_grid_sdf_adapter_honors_sdf_sign_convention` (lines
    320-349)
- Verify: `cargo build -p mesh-sdf` + `cargo test -p mesh-sdf` green.

**Estimate**: ~20 min.

### Phase 3 — cf-design adopts cf-sdf (drops trait + adapters)

- `cf-design/Cargo.toml`: add `cf-sdf = { workspace = true }`.
- `cf-design/src/sdf.rs`:
  - DELETE trait def + Box/Arc impls + mesh-sdf adapter impls
    (lines 1-69 + 113-158).
  - KEEP `impl Sdf for Solid` (lines 77-85) — needs the `Solid`
    type which stays in cf-design.
  - Adjust imports: `use cf_sdf::Sdf;` at the top.
- `cf-design/src/lib.rs`: `pub use cf_sdf::Sdf;` for backward compat
  re-export. All 13 external consumers of `cf_design::Sdf` (per
  recon §2.5) keep working unchanged.
- Tests that stay in cf-design:
  - `solid_sphere_eval_sign_convention` (lines 165-182)
  - `solid_sphere_grad_unit_outward_on_surface` (lines 184-197)
  - `solid_csg_difference_eval_at_hollow_shell_probes` (lines
    199-222)
- Verify: `cargo build -p cf-design` + `cargo test -p cf-design` green.

**Estimate**: ~15 min.

### Phase 4 — sim-soft swaps cf-design → cf-sdf

- `sim/L0/soft/Cargo.toml`: drop `cf-design` direct dep; add
  `cf-sdf = { workspace = true }`. Update comment (the Q5 inventory
  rationale).
- `sim/L0/soft/src/sdf_bridge/sdf.rs:44`: `pub use cf_design::Sdf;`
  → `pub use cf_sdf::Sdf;`.
- Verify:
  - `cargo build -p sim-soft` + `cargo test -p sim-soft --release
    --no-default-features --lib` green (151 tests, no regressions).
  - `cargo tree -p sim-soft -i cf-design` returns nothing (cf-design
    fully gone from sim-soft's tree).

**Estimate**: ~10 min.

### Phase 5 — Layer-integrity close: tracing → log in sim-core

- `sim/L0/core/Cargo.toml`: drop `tracing` dep; add
  `log = { workspace = true }` (which already exists in workspace
  deps as a transitive — confirm with `grep '^log' Cargo.toml`).
- Rewrite the 4 sim-core sites:
  - `types/warning.rs:63`: `tracing::warn!(...)` → `log::warn!(...)`
  - `forward/check.rs:100`: `tracing::error!(...)` → `log::error!(...)`
  - `forward/mod.rs:177`: `tracing::warn!(...)` → `log::warn!(...)`
  - `collision/mesh_collide.rs:15`: `use tracing::warn;` →
    `use log::warn;`
- Macro syntax is largely compatible; one or two sites may need
  format-string adjustments (tracing supports structured kv fields
  via `field = value`, log doesn't — substitute with `format!`).
- Verify: `cargo build -p sim-core` + `cargo test -p sim-core` +
  `cargo build -p sim-soft` all green.

**Estimate**: ~15 min.

### Phase 6 — Verify the close

- `cargo xtask grade sim-soft --skip-coverage`:
  - Documentation A, Clippy A, Safety A, Dependencies A, WASM A.
  - Layer Integrity: **A** (dep count ≤ 99, under 100 max).
- `cargo xtask grade {cf-sdf, cf-design, mesh-sdf, sim-core,
  sim-ml-chassis, cf-cast-cli, cf-device-design, cf-device-types,
  cf-device-geometry, cf-sim-research, cf-scan-prep}` — all
  Automated A.
- `cargo test --workspace --release` clean (or sampled if too long).
- Update `docs/PR_249_REVIEW.md` B2 verdict from "Layer Integrity F
  pre-existing" → "RESOLVED via cf-sdf trait-split + tracing→log
  migration".

**Estimate**: ~10 min.

---

## 4. Total estimate

- Phase 1: 30 min
- Phase 2: 20 min
- Phase 3: 15 min
- Phase 4: 10 min
- Phase 5: 15 min
- Phase 6: 10 min

**Total**: 90-100 min, single session.

**Commit shape**: per [[feedback-omnibus-pr-single-branch]] this
arc is one logical change (architectural surgery + the cut needed
to close the gap it surfaced). One commit covers all six phases.
Alternative: split Phase 5 (tracing→log) into its own commit since
it's orthogonal to the trait-split — easier audit-trail for the
two distinct cuts.

Default lean: **two commits**:
1. Trait-split (Phases 1-4) — `refactor(cf-sdf): lift Sdf trait
   from cf-design — sim-soft drops design-kernel dep`.
2. tracing→log close (Phase 5) — `refactor(sim-core): tracing →
   log — layer-integrity close on sim-soft`.

Both land on `dev`, appending to PR #249's omnibus.

---

## 5. Risks + mitigations

### R1 — `cf-sdf` name collision

Workspace already has `cf-design`, `cf-design-types`, `cf-design-tests`,
`cf-device-design`, `cf-device-types`, `cf-device-geometry`. The name
`cf-sdf` is unused (no crate, no `[workspace.dependencies]` entry).
Confirmed by `ls design/` + `grep "cf-sdf" Cargo.toml` showing zero
hits.

**Mitigation**: none needed. If the user prefers a different name
at impl time (`cf-sdf-trait`, `sdf-core`, `cf-sdf-contract`), trivial
to rename — single crate, single use site to update.

### R2 — Orphan-rule constraint on the mesh-sdf adapter move

`impl Sdf for Signed<D, S>` and `impl Sdf for CachedGridSdf` are
currently in cf-design because cf-design owned the trait and the
orphan rule blocked mesh-sdf from impl-ing for its own types
without the trait being in scope. After Phase 2, mesh-sdf
depends on cf-sdf, so it can host these impls directly. **Phase 1
must finish before Phase 2.**

**Mitigation**: phase ordering pins this. The compiler enforces it
— if mesh-sdf tries to impl `cf_sdf::Sdf for Signed` without
depending on cf-sdf, rustc errors with "trait not in scope".

### R3 — `Sdf` is `pub use` re-exported from cf-design's `lib.rs`

Backward compat for the 13 external consumers depends on a
`pub use cf_sdf::Sdf;` at cf-design's `lib.rs`. If this line is
forgotten in Phase 3, every consumer of `cf_design::Sdf` breaks at
compile time.

**Mitigation**: Phase 3 verification step is `cargo test --workspace`
— catches this immediately via the consumer-side compile errors.

### R4 — tracing → log macro semantic drift

`tracing::warn!(field = value, "msg")` (structured kv) and
`log::warn!("msg: {value}")` (formatted string) aren't identical.
The 4 sim-core sites need a brief read to confirm the calls aren't
relying on kv fields. From recon §2.4 these are simple `warn!(...)`
calls with format-string args; the kv-field pattern is rare in
sim-core.

**Mitigation**: Phase 5 spec says read each site before replacing;
fall back to `log::warn!(format!("..."))` if a site uses kv fields
in a way `log` can't express.

### R5 — Workspace dep on `log` not in `[workspace.dependencies]`

Earlier `grep "^log " Cargo.toml` returned empty — log might not be
a workspace dep entry, just a transitive. sim-core's Cargo.toml
would need to declare it as a direct dep, possibly as
`log = "0.4"` (matching the transitive resolved version).

**Mitigation**: Phase 5 spec checks `[workspace.dependencies]`;
adds `log = "0.4"` (or whatever version resolves) if missing.

### R6 — sim-soft tests use `cf_design::*` for fixtures

Spot-checked sim-soft's lib tests — they import from `cf_design`
for `Solid::sphere` etc. in fixture construction. These would
break when sim-soft drops cf-design as a direct dep.

**Mitigation**: If found at impl time, add `cf-design = { workspace
= true }` as a **dev-dependency** (release graph stays slim;
test graph gets cf-design back for fixtures). Verified pattern
in cf-design-tests's `dev-dependencies` block.

---

## 6. What stays at parity

Post-arc:

- All 13 external `cf_design::Sdf` consumers (mesh-sdf, cf-cast,
  cf-device-geometry, cf-design itself, cf-sim-research, cf-cast-cli,
  5 examples, sim-soft) continue working — sim-soft via direct
  `cf_sdf::Sdf`, the other 12 via cf-design's re-export.
- cf-design's user-facing API: `Solid`, CSG ops, adaptive mesher
  — unchanged.
- mesh-sdf's user-facing API: `TriMeshDistance`, `Signed`,
  `CachedGridSdf`, `FloodFillSign`, etc. — unchanged.
- Workshop loop: scan → cf-device-design → TOML → cf-cast-cli → mold
  STLs — unchanged.
- Sim research loop: load design TOML + scan in cf-sim-research →
  run sim — unchanged.
- sim-soft's API: Material, Element, Solver, Contact, sdf_bridge —
  unchanged.

---

## 7. Pointers

- `docs/PR_249_REVIEW.md` §B2 — the layer-integrity gap surfaced
- Commit `9eb6cd59` — Phase 0 work (rayon gate, 5 deps dropped)
- `design/cf-design/src/sdf.rs` — current host of the `Sdf` trait
  + the four impls + the test fixture
- `sim/L0/soft/src/sdf_bridge/sdf.rs:44` — the single sim-soft
  source-side change in Phase 4
- `sim/L0/core/src/{types/warning.rs:63, forward/check.rs:100,
  forward/mod.rs:177, collision/mesh_collide.rs:15}` — the 4
  tracing call sites for Phase 5
- `docs/PROGRAM_GAMEPLAN.md` §7.1 — research-cluster vs product-
  pipeline crate map; sim-soft's L0 placement
- Three-session pattern memo per
  [[feedback-bookmark-when-surface-levers-exhaust]] — this recon
  is session 2 of 3 (session 1 = the bookmark via PR_249_REVIEW
  + commit `9eb6cd59`; session 3 = implementation per §3).

---

End of recon. **NEXT SESSION**: implement per §3. Both commits
land on `dev` and append to PR #249 per
[[feedback-omnibus-pr-single-branch]].
