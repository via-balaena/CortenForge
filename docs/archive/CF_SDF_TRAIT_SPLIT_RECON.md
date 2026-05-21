# Sdf trait migration + sim-soft layer-integrity close — RECON

> **Status**: RECON SHIPPED 2026-05-20, cold-read REVISED same session
> (math correction + architectural pivot from new `cf-sdf` crate to
> existing `cf-geometry` host). Implementation in the next session,
> intended to land on `dev` and append to PR #249.
>
> **Triggered by**: PR #249's CI layer-integrity failure for sim-soft
> (108 transitive deps > L0 tier max 100). Phase 0 fix in commit
> `9eb6cd59` gated cf-design's `parallel-meshing` feature; sim-soft
> opts out → 103 deps, still 3 over.
>
> **Predecessors**:
> - `docs/archive/PR_249_REVIEW.md` §B2 — surfaces the layer-integrity gap
> - Commit `9eb6cd59` — Phase 0 work (rayon gate, 5 deps dropped)

---

## TL;DR

Move cf-design's `Sdf` trait into the existing `cf-geometry` crate
(already an L0 workspace member sim-soft pulls transitively). cf-design
keeps `impl Sdf for Solid`; mesh-sdf gains `impl Sdf for Signed<D, S>`
+ `impl Sdf for CachedGridSdf` (currently orphan-rule-parked in
cf-design). sim-soft swaps from depending on cf-design (the design
kernel) to depending only on the trait contract via cf-geometry —
which it already pulls. Pair with replacing sim-core's `tracing`
usage (4 dead-weight call sites) with `log`.

Combined effect on sim-soft:
- Original baseline (main): 108 deps.
- After commit `9eb6cd59` (rayon gate): 103.
- After this arc: **~99**, Layer Integrity A.

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
mesher, `mesh_simplified` / `mesh_to_tolerance`, rayon (now gated),
etc. That's a dep-direction smell: a foundational solver crate
should NOT depend on a design-side kernel that USES it.

### 1.2 The right architecture

Sdf belongs in cf-geometry (the L0 "geometric kernel" crate that
already houses `Aabb`, `Bvh`, `ConvexHull`, `Sphere`, `Triangle`,
`SdfGrid` — and that already has zero deps beyond nalgebra).

```
cf-geometry       — Geometric primitives + the `Sdf` TRAIT contract
                    (new). Box/Arc blanket impls live here. Deps:
                    nalgebra. Already L0, already on sim-soft's path.
cf-design         — Solid + CSG + adaptive mesher + (kept) impl Sdf
                    for Solid. Re-exports `pub use cf_geometry::Sdf`
                    for the 10 external consumers that import via
                    cf-design today.
mesh-sdf          — TriMeshDistance + Sign oracles + (moved) impl
                    Sdf for `Signed<D, S>` + impl Sdf for
                    `CachedGridSdf` (these were parked in cf-design
                    because of the orphan rule; now resolved).
```

Sim-soft depends on cf-geometry (which it already does transitively
via mesh-types), no longer on cf-design. No new workspace crate
created.

---

## 2. Recon findings

### 2.1 The `Sdf` trait surface is tiny

`design/cf-design/src/sdf.rs` (351 LOC total):

| Lines | Content | Migrates to |
|---|---|---|
| 1-13 | module docstring | adapt + lift |
| 14-26 | `use` statements | rewrite |
| 28-34 | `pub trait Sdf` (7 LOC) | **cf-geometry** |
| 42-50 | `impl<T: Sdf + ?Sized> Sdf for Box<T>` | **cf-geometry** |
| 61-69 | `impl<T: Sdf + ?Sized> Sdf for Arc<T>` | **cf-geometry** |
| 77-85 | `impl Sdf for Solid` (8 LOC) | stays in cf-design |
| 113-137 | `impl<D, S> Sdf for Signed<D, S>` (25 LOC) | **mesh-sdf** |
| 150-158 | `impl Sdf for CachedGridSdf` (9 LOC) | **mesh-sdf** |
| 160-350 | tests | partition by destination |

cf-geometry lift surface: **~30 LOC of trait + blanket impls** plus
2 tests (Box + Arc forwarding). Deps required: nalgebra
(`Point3<f64>`, `Vector3<f64>`) — already cf-geometry's only direct
dep.

### 2.2 cf-design's contribution to sim-soft's tree is the cf-design crate itself

**Key recon measurement** (exclusive-path enumeration via
sequential `cargo tree -i $dep`):

| Dep cf-design contributes to sim-soft | Also reachable via sim-soft's OTHER direct deps? |
|---|---|
| All 57 transitive items (minus cf-design itself) | **Yes — every one** |
| cf-design itself | No (only path is sim-soft's direct dep) |

The intuition "cf-design contributes 57 unique transitive deps →
removing it drops 57" was wrong. cf-design's transitive set is
fully overlapped with sim-soft's other direct deps (mesh-offset →
mesh-sdf → parry3d/nalgebra/etc.; sim-ml-chassis → sim-core →
tracing/serde/etc.; mesh-types → cf-geometry/nalgebra). Removing
cf-design as a direct dep removes only `cf-design` itself = **1 dep**.

### 2.3 Why cf-geometry is the right home (not a new cf-sdf crate)

The original recon proposed creating a new `cf-sdf` crate. That
would have netted **0** dep savings (drop cf-design = -1 + add
cf-sdf = +1). cf-geometry as the home is strictly better:

- cf-geometry already exists, already L0, already on sim-soft's
  transitive path (via mesh-types → cf-geometry; via mesh-offset
  → mesh-sdf → mesh-types → cf-geometry). Adding the trait to
  cf-geometry contributes **zero new transitive deps** to anyone.
- cf-geometry's current scope ("geometric kernel — Aabb / Bvh /
  Sphere / Triangle / `SdfGrid`") is a natural home for the
  signed-distance-function trait. Putting the trait next to
  `SdfGrid` (the concrete uniform-grid SDF storage) consolidates
  the SDF concept in one module.
- One fewer workspace member to grade + carry, one less Cargo.toml
  + lib.rs surface.
- **Net dep change**: drop cf-design from sim-soft direct = **-1**.

Optional polish (not in the arc, banked): impl Sdf for SdfGrid in
cf-geometry — orphan-rule-clean since both are cf-geometry-owned.
Not load-bearing for sim-soft (sim-soft doesn't use SdfGrid), so
defer.

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
ena → log`). Swapping sim-core's tracing for log adds zero new
deps. Migration is mechanical (~6 LOC across 4 sites).

**Combined savings**: trait-move (1) + tracing→log (3) = **4 deps**.
sim-soft lands at **103 - 4 = 99** → Layer Integrity A.

### 2.5 cf_design::Sdf consumer enumeration

14 source files reference `cf_design::Sdf` directly (`use`, type
alias, or trait-bound). Breakdown by Phase impact:

| Category | Sites | Phase impact |
|---|---:|---|
| mesh-sdf adapter hosts (`oracle.rs`, `lib.rs`) | 2 | adapter impls MOVE here (Phase 2) |
| sim-soft migration target (`sdf_bridge/sdf.rs:44`) | 1 | swap to `cf_geometry::Sdf` (Phase 4) |
| cf-design self-reference (`solid_layered.rs`) | 1 | adjust import to `crate::Sdf` (Phase 3) |
| External consumers — keep working via re-export | 10 | no change required (cf-cast, cf-device-geometry, cf-sim-research, cf-cast-cli ×2, 5 examples) |
| **Total** | **14** | |

cf-design's `lib.rs` adds `pub use cf_geometry::Sdf;` to keep the
10 external consumers' `cf_design::Sdf` imports compiling. Only
sim-soft is actively migrated to the new path.

---

## 3. Migration plan

Five phases. Each ends with a green workspace build.

### Phase 1 — cf-geometry gains the `Sdf` trait

- `design/cf-geometry/src/sdf.rs`: extend (don't replace — `SdfGrid`
  stays untouched). Add:
  - Module docstring section "## Sdf trait" describing the
    contract semantics (sign convention, gradient meaning,
    Send+Sync rationale) — lifted from cf-design/src/sdf.rs:1-27.
  - `pub trait Sdf: Send + Sync { fn eval; fn grad; }` (verbatim
    from cf-design lines 28-34).
  - `impl<T: Sdf + ?Sized> Sdf for Box<T>` (verbatim from cf-design
    lines 42-50).
  - `impl<T: Sdf + ?Sized> Sdf for Arc<T>` (verbatim from cf-design
    lines 61-69).
- Re-export from `cf-geometry/src/lib.rs`: add `pub use sdf::Sdf;`
  (or follow the crate's existing pub-use pattern).
- Tests added to `cf-geometry/src/sdf.rs`'s test mod:
  - `box_dyn_sdf_blanket_forwards_eval_and_grad` (cf-design lines
    225-236) — adjusted to use a tiny in-test `ConstSdf` fixture
    instead of `Solid::sphere` (cf-geometry has no cf-design dep).
  - `arc_dyn_sdf_blanket_forwards_eval_and_grad` (lines 238-255)
    — same fixture adjustment.
- Verify: `cargo build -p cf-geometry`, `cargo test -p cf-geometry`,
  `cargo xtask grade cf-geometry --skip-coverage` — all green.

**Estimate**: ~30 min.

### Phase 2 — mesh-sdf adopts cf-geometry::Sdf for its adapter impls

- `mesh-sdf/Cargo.toml`: confirm cf-geometry is already a workspace
  dep (it is, transitively at minimum via mesh-types — verify
  direct dep or add it).
- Move `impl Sdf for Signed<D, S>` (cf-design/src/sdf.rs:113-137)
  → `mesh-sdf/src/sdf_adapter.rs` (new file, or appended to
  `oracle.rs` / `lib.rs` — recon pass during impl picks the home).
  Change trait reference to `cf_geometry::Sdf`.
- Move `impl Sdf for CachedGridSdf` (lines 150-158) → same.
- Add module-level pub re-export `pub use cf_geometry::Sdf;` to
  `mesh-sdf/src/lib.rs` for downstream `use mesh_sdf::Sdf` ergonomics
  (optional polish; either path resolves the trait).
- Tests migrate with the impls:
  - `mesh_sdf_eval_sign_convention_in_contact_band` (cf-design
    lines 284-298)
  - `mesh_sdf_grad_below_bottom_face_approximates_outward_normal`
    (lines 300-312)
  - `cached_grid_sdf_adapter_honors_sdf_sign_convention` (lines
    320-349)
  - Plus the `unit_tetrahedron` fixture (cf-design lines 260-271)
    and the `mesh_sdf_for_tetrahedron` builder (lines 277-282) —
    both move with their callers.
- Verify: `cargo build -p mesh-sdf`, `cargo test -p mesh-sdf`,
  grade green.

**Estimate**: ~25 min.

### Phase 3 — cf-design adopts cf-geometry::Sdf

- `cf-design/Cargo.toml`: cf-geometry is already a direct dep
  (`design/cf-design/Cargo.toml` line 19). No Cargo.toml change.
- `cf-design/src/sdf.rs`:
  - DELETE: trait def (lines 28-34), Box/Arc impls (lines 42-50,
    61-69), and the two mesh-sdf adapter impls (lines 113-137 + 150-
    158). Also delete the tests that moved with them (Box/Arc tests
    moved to cf-geometry; mesh-sdf adapter tests moved to mesh-sdf).
  - KEEP: `impl Sdf for Solid` (lines 77-85) — the canonical
    Solid-side impl. Plus its tests (lines 165-222).
  - Adjust top-of-file imports: `use cf_geometry::Sdf;`. Drop
    unused mesh-sdf type imports if they only supported the moved
    adapters.
- `cf-design/src/lib.rs`: add `pub use cf_geometry::Sdf;` so
  external consumers' `cf_design::Sdf` imports keep compiling.
  This is the backward-compat seam — covers 10 external sites
  per §2.5.
- `cf-design/src/solid_layered.rs`: adjust whatever `use` line
  imported `crate::Sdf` (now re-exported); verify a clean
  in-crate path or update to `crate::Sdf` via the lib.rs re-export.
- Verify: `cargo build -p cf-design`, `cargo test -p cf-design`,
  grade green. **Critical check**: `cargo build --workspace` to
  catch any of the 10 external consumers that broke.

**Estimate**: ~20 min.

### Phase 4 — sim-soft swaps cf-design → cf-geometry direct dep

- `sim/L0/soft/Cargo.toml`:
  - DROP `cf-design = { workspace = true }` (the entry that triggered
    the Layer Integrity F).
  - ADD `cf-geometry = { workspace = true }` (sim-soft currently
    pulls cf-geometry transitively; this makes it direct — needed
    so `use cf_geometry::Sdf` resolves at sim-soft's call sites).
  - Update the inline comment (currently mentions cf-design's Sdf
    re-export — rewrite to point at cf-geometry).
- `sim/L0/soft/src/sdf_bridge/sdf.rs:44`: change
  `pub use cf_design::Sdf;` → `pub use cf_geometry::Sdf;`.
- The docstring sites in `sdf_bridge/{sdf.rs, difference.rs}` that
  mention `cf_design::Solid::sphere` etc. — leave alone. Those are
  docstring references to cf-design's user-facing surface for the
  README-style narrative, not actual code dependencies; the prose
  still applies (cf-design still ships `Solid` and is still where
  users build CSG SDFs).
- Verify:
  - `cargo build -p sim-soft` + `cargo test -p sim-soft --release
    --no-default-features --lib` → 151 tests green (no regressions).
  - `cargo tree -p sim-soft -i cf-design` returns "no matches"
    (cf-design fully gone from sim-soft's tree).
  - `cargo xtask grade sim-soft --skip-coverage`: dep count down
    to 102 (was 103; trait-move saved 1).

**Estimate**: ~15 min.

### Phase 5 — sim-core tracing → log (the Layer-Integrity close)

- `sim/L0/core/Cargo.toml`:
  - DROP `tracing = { workspace = true }`.
  - ADD `log = "0.4"` (or whatever version the workspace transitive
    resolves to). Optional follow-up: add `log = "0.4"` to
    `[workspace.dependencies]` in the root `Cargo.toml` so future
    consumers can use `log = { workspace = true }` — but not
    required to close this arc.
- Rewrite the 4 sim-core sites:
  - `types/warning.rs:63`: `tracing::warn!("{} Time = {:.4}.", ...)`
    → `log::warn!("{} Time = {:.4}.", ...)` (mechanical).
  - `forward/check.rs:100`: `tracing::error!(...)` → `log::error!(...)`.
  - `forward/mod.rs:177`: `tracing::warn!(...)` → `log::warn!(...)`.
  - `collision/mesh_collide.rs:15`: `use tracing::warn;` →
    `use log::warn;`.
- **Macro-syntax caveat**: tracing supports structured `field = value`
  kv args inside the macro call; `log` doesn't. Read each site
  first; if any uses kv fields (none currently appear to — they're
  plain `tracing::warn!("msg", args...)`), substitute the kv with
  a `format!`-baked string. Phase verification compiles the crate,
  so this surfaces immediately if missed.
- Verify: `cargo build -p sim-core`, `cargo test -p sim-core`,
  `cargo build -p sim-soft`, `cargo xtask grade sim-soft
  --skip-coverage` → **Layer Integrity A** (dep count 99 ≤ 100).

**Estimate**: ~20 min.

### Verification gate (built into Phase 5)

- `cargo xtask grade sim-soft --skip-coverage`: all criteria A
  (Doc / Clippy / Safety / Dependencies / WASM / Layer Integrity).
- `cargo xtask grade {cf-geometry, cf-design, mesh-sdf, sim-core,
  sim-ml-chassis, cf-cast-cli, cf-device-design, cf-device-types,
  cf-device-geometry, cf-sim-research, cf-scan-prep}` — all A.
- `cargo test --workspace` sample (or `--release` for sim-soft).
- Update `docs/archive/PR_249_REVIEW.md` §B2 verdict: "Layer Integrity F
  pre-existing" → "RESOLVED via Sdf trait migration to cf-geometry
  + tracing→log close".

---

## 4. Total estimate

- Phase 1: 30 min
- Phase 2: 25 min
- Phase 3: 20 min
- Phase 4: 15 min
- Phase 5: 20 min

**Total**: 90-110 min, single session.

**Commit shape**: two commits per [[feedback-omnibus-pr-single-branch]]
for audit-trail clarity (the two cuts are orthogonal):

1. `refactor(cf-geometry): lift Sdf trait from cf-design — sim-soft
   drops design-kernel dep` (Phases 1-4 in a single atomic commit;
   each phase compiles, but landing as one commit avoids transient
   workspace-build breakage from half-state imports).
2. `refactor(sim-core): tracing → log — layer-integrity close on
   sim-soft` (Phase 5).

Both land on `dev`, appending to PR #249.

---

## 5. Risks + mitigations

### R1 — Phase 3 backward-compat re-export drift

If the `pub use cf_geometry::Sdf;` line in cf-design's lib.rs is
missed, the 10 external consumers' `cf_design::Sdf` imports break
compile-wide.

**Mitigation**: Phase 3 verification runs `cargo build --workspace`
explicitly — catches all 10 consumers' breaks immediately. The fix
is single-line.

### R2 — Phase 2 orphan-rule sequence

`impl Sdf for Signed<D, S>` and `impl Sdf for CachedGridSdf` move
to mesh-sdf. mesh-sdf must depend on cf-geometry (where Sdf now
lives) at compile time of those impls. **Phase 1 must finish
before Phase 2.**

**Mitigation**: rustc enforces ordering — if mesh-sdf tries to impl
`cf_geometry::Sdf` without depending on cf-geometry, it errors
with "trait not in scope". Verify cf-geometry is in mesh-sdf's
Cargo.toml before writing the impl.

### R3 — tracing → log macro semantic drift

`tracing::warn!(field = value, "msg")` (structured kv) and
`log::warn!("msg: {value}")` (formatted string) aren't identical.
The 4 sim-core sites need a brief read to confirm the calls aren't
relying on kv fields. From recon §2.4 these are plain `warn!(...)`
calls with positional format-string args; the kv-field pattern is
absent in sim-core.

**Mitigation**: Phase 5 spec says read each site before replacing.
If a site uses kv fields, fall back to `log::warn!(&format!("..."))`
— still drops the tracing chain.

### R4 — `log` not in workspace.dependencies

`grep "^log " Cargo.toml` returns empty — `log` is currently
pulled only transitively (via parry3d's `ena`). sim-core's
Cargo.toml needs to specify a version directly (e.g., `log = "0.4"`)
rather than `{ workspace = true }`.

**Mitigation**: Phase 5 spec uses direct version pin. Optional
polish: add `log = "0.4"` to `[workspace.dependencies]` for future
consumers — not required to close this arc, but ~3 LOC away if
desired.

### R5 — Phase 4 sim-soft test-fixture surprise (downgraded from prior recon)

The prior recon flagged "sim-soft tests might use `cf_design::Solid::sphere`
for fixtures". Verified at recon time: sim-soft has **zero `use
cf_design::*` in lib code or tests**, only 9 docstring references
in `sdf_bridge/{sdf.rs, difference.rs}` mentioning cf-design's
public surface for the README-style narrative. No fixture dep.

**Mitigation**: none needed. R5 reduced to a smoke check at impl
time ("grep `cf_design::` in sim-soft's src, confirm zero `use`
matches before the swap").

### R6 — cf-geometry's `Sdf` trait + `SdfGrid` naming proximity

cf-geometry already has `SdfGrid` (concrete grid-storage SDF
struct). Adding `Sdf` (the trait) means cf-geometry exports both
`Sdf` and `SdfGrid`. Cognitive load is mild but real ("which is the
contract, which is the impl?").

**Mitigation**: extend `cf-geometry/src/sdf.rs`'s module docstring
to call out the relationship explicitly — `Sdf` is the contract,
`SdfGrid` is one concrete in-crate type that COULD implement it
(but doesn't today; banked as optional polish in §2.3). Anyone
reading the file gets the answer immediately.

---

## 6. What stays at parity

Post-arc:

- All 10 external `cf_design::Sdf` consumers (cf-cast,
  cf-device-geometry, cf-sim-research, cf-cast-cli ×2, 5 examples)
  continue working unchanged via cf-design's re-export.
- mesh-sdf's 2 adapter impls move (functional behavior identical;
  trait reference changes from `cf_design::Sdf` to `cf_geometry::Sdf`).
- cf-design's user-facing API: `Solid`, CSG ops, adaptive mesher
  — unchanged. `cf_design::Sdf` still works (re-export).
- mesh-sdf's user-facing API: `TriMeshDistance`, `Signed`,
  `CachedGridSdf`, `FloodFillSign`, etc. — unchanged.
- cf-geometry gains an export surface (`Sdf` trait + Box/Arc impls)
  — additive, no removal.
- Workshop loop: scan → cf-device-design → TOML → cf-cast-cli → mold
  STLs — unchanged.
- Sim research loop: load design TOML + scan in cf-sim-research →
  run sim — unchanged.
- sim-soft's API: Material, Element, Solver, Contact, sdf_bridge —
  unchanged. `sim_soft::sdf_bridge::Sdf` still re-exported (just
  now from cf-geometry, not cf-design).

---

## 7. Pointers

- `docs/archive/PR_249_REVIEW.md` §B2 — the layer-integrity gap surfaced
- Commit `9eb6cd59` — Phase 0 work (rayon gate, 5 deps dropped)
- `design/cf-design/src/sdf.rs` — current host of `Sdf` trait +
  the four impls + tests
- `design/cf-geometry/src/sdf.rs` — current host of `SdfGrid`;
  the trait + blankets get appended here in Phase 1
- `design/cf-geometry/src/lib.rs` — gets `pub use sdf::Sdf;` in
  Phase 1
- `design/cf-design/src/lib.rs` — gets `pub use cf_geometry::Sdf;`
  re-export in Phase 3 for the 10 external consumers
- `sim/L0/soft/src/sdf_bridge/sdf.rs:44` — single sim-soft
  source-side change in Phase 4
- `sim/L0/core/src/{types/warning.rs:63, forward/check.rs:100,
  forward/mod.rs:177, collision/mesh_collide.rs:15}` — 4 tracing
  call sites for Phase 5
- `docs/PROGRAM_GAMEPLAN.md` §7.1 — research-cluster vs
  product-pipeline crate map; sim-soft's L0 placement
- Three-session pattern memo per
  [[feedback-bookmark-when-surface-levers-exhaust]] — this recon
  is session 2 of 3 (session 1 = bookmark via `PR_249_REVIEW` +
  commit `9eb6cd59`; session 3 = implementation per §3).

---

End of recon. **NEXT SESSION**: implement per §3 (5 phases).
Both commits land on `dev` and append to PR #249 per
[[feedback-omnibus-pr-single-branch]].
