# PR #249 holistic review

> Reviewed: 2026-05-20, single session (findings density turned out low — pre-budget was 2-4 sessions).
> Scope: cumulative diff `dev` → `main`, 109 commits, +21 165 / −1 988 LOC, 65 changed files.
> Posture: catalog only, no fixes. Per [[feedback-omnibus-pr-single-branch]] this is the close of three concurrent arcs (A1 sim-decouple + B1 mold-wall + B2 roadmap) plus the historic F3/F4/H2/H4/CR/SL recon commits along for the ride.

---

## Verdict

- [x] **Ready to merge as-is** — `xtask grade` (CI-blocking) flagged 9 crates F-grade on `d0a02ad`; all resolved. 8 of 9 closed in `291d6df8` + `958f1e92`; sim-soft Layer Integrity closed via the Sdf-trait-split arc (`9eb6cd59` + `1b8dbc9e` + `6c8572ec`). sim-soft now Automated A across all 7 criteria. See B2 below.
- [ ] Ready to merge after Block-class fixes
- [ ] Needs another arc before merge

**Original review's Block count was 0 — wrong.** The initial pass verified architecture invariants (`cargo tree`, `cargo build`, direct-dep grep) but did NOT run `cargo doc` with `-D warnings` or `cargo xtask grade`. Both are CI-load-bearing on this workspace per STANDARDS.md and both flagged real issues. Lesson for future reviews: `cargo clippy / cargo build` ≠ "all checks run"; xtask grade exists and matters. Added to the review doc itself as the post-mortem note.

---

## Block-class findings

### B1 — xtask grade F-grade on 5 PR-introduced crates @ CI `d0a02ad` — RESOLVED in `291d6df8`

CI flagged xtask grade F on `d0a02ad` (the A1 Phase 5 close commit) for 9 workspace crates. 5 of those were PR-introduced and addressed in commit `291d6df8` (8 specific fixes — 7 doc-link surfaces moved by Phase 1/2.5/3 lifts whose qualifiers got stale, 1 HTML-tag-in-quoted-string warning, 2 unjustified-clippy-allow comments). See the commit message for the full site list.

This finding belonged in the original review and was missed because the architecture sweep didn't run `cargo doc -- -D warnings` or `cargo xtask grade`. Lesson banked.

### B2 — xtask grade F-grade on sim-soft + 3 examples — RESOLVED in `958f1e92` (doc) + `9eb6cd59` + `1b8dbc9e` + `6c8572ec` (layer integrity)

Same CI failure, second class. 4 more crates failing — addressed in commit `958f1e92`:

- **cf-scan-prep / example-mesh-mesh-sdf-distance-query / example-sim-soft-mesh-scan-as-solid** — 1 doc warning each, ALL pre-existing on `main`. Bundled with the sim-soft doc fixes since they're trivial backtick conversions. Now grade A.
- **sim-soft Documentation** — 12 doc warnings, PR-introduced (sim-soft grades Documentation A on `main`). All from private-item intra-doc links inside the F3/CR/E.b.* paused-recon commits. Backtick conversion. Now grade A.
- **sim-soft Layer Integrity** — was F: `dep count 108 exceeds max 100` for L0 tier. **RESOLVED via Sdf trait migration to cf-geometry + tracing→log close** (commits `9eb6cd59` rayon-gate + `1b8dbc9e` trait migration + `6c8572ec` tracing→log). The trait migration is the architectural fix: a foundational L0 solver (sim-soft) was depending on the design-side kernel (cf-design) that uses it. Moving the trait to cf-geometry (which sim-soft already pulls transitively as the geometric kernel) lets sim-soft consume the contract without the design-kernel weight. The tracing→log swap drops three more transitive deps that paid full cost for output going to `/dev/null` (no `tracing-subscriber` is installed workspace-wide). Final distinct release-deps for sim-soft: 98 (≤100), Layer Integrity A.

**PR #249 merge decision** unblocked — sim-soft now Automated A across all 7 criteria. Block-class B2 fully resolved.

---

## Polish-class findings

### P1 — Shared-crate docstrings name only sim consumers @ `design/cf-device-geometry/src/bevy_mesh.rs:9`

The `bevy_mesh` module docstring frames `build_bevy_mesh_from_indexed*` as the adapter "for the deformed-cavity / intruder render in the sim panel" — both sim-research-only consumers. cf-device-design also consumes this adapter (rest-frame cavity + per-iso layer meshes via `extract_layer_surface`), but the docstring doesn't acknowledge that consumer. Symmetric drift to the Phase 5 finding, opposite direction (CAD-side use under-represented in shared docstring). Polish.

Suggested rewrite: mention BOTH consumers, e.g. "consumed by cf-device-design (rest-frame cavity + per-iso layer meshes) and cf-sim-research (deformed-cavity + intruder + heat-map variant)".

### P2 — `sdf_layers.rs` "future uses" docstring asymmetry @ `design/cf-device-geometry/src/sdf_layers.rs:177-180`

The `CachedScanSdf` docstring on the retained `sdf_closed`/`sdf_open`/`bounds` fields names two future uses: "(a) the heat-map re-projection (per-tet → per-layer-vertex closest-point lookup) and (b) a higher-fidelity Save path. `bounds` is held for the same future Save-side consumer". Heat-map re-projection is cf-sim-research-only; Save path is cf-device-design-only. So the docstring DOES touch both binaries — that's fine for a shared crate — but it reads as if the same future consumer needs both. Tighten the wording to clarify they're separate consumers (one per binary).

### P3 — Cross-references to `insertion_sim::*` in shared geometry crate @ `design/cf-device-geometry/src/sdf_layers.rs:18, 143, 254, 260, 266, 776, 815`

Seven sites in `sdf_layers.rs` reference `insertion_sim::decimate_for_sdf` / `insertion_sim::build_insertion_geometry` / `insertion_sim_ui` as if those modules live in the same crate. They live in `tools/cf-sim-research/src/`. Each reference is internal documentation (not user-facing) and each is factually accurate, but cold-reading the shared crate gives the impression `insertion_sim` is a close library peer. Worth qualifying as `cf-sim-research's insertion_sim::*` at each site, or pulling the cross-reference up to a single docstring block in the module header that explains the relationship once.

### P4 — One stale spec-doc cross-ref in cf-device-geometry @ `design/cf-device-geometry/src/clip_plane.rs:1` and `sdf_layers.rs:?` (search for `docs/CF_DEVICE_DESIGN_*_SPEC.md`)

Two `docs/CF_DEVICE_DESIGN_{CLIP_PLANE,SDF_LAYERS}_SPEC.md` references in code that now lives in shared `cf-device-geometry/`. Per Phase 4 SHIPPED block this is intentional audit-trail (design-of-record predates the Phase 2.5 lift). NOT a finding — flagged here only because a cold-reader could plausibly want to rename the docs. Keep as-is per the banked posture.

---

## Note-class findings

### N1 — `cf-device-types::sim` is consumed ONLY by cf-sim-research

`SimDesign`, `SimLayer`, `SimMode`, `ScalarMode`, `SlackerResolution` (the entire `sim` submodule of cf-device-types) have **77 uses in cf-sim-research and 0 uses in cf-device-design**. The lib.rs docstring frames cf-device-types as a shared crate "so cf-device-design + cf-sim-research describe a layered-silicone device the same way" — but cf-device-design doesn't describe the sim-side projection. By that framing, `sim.rs` is sim-research-internal types parked in a shared crate.

Three options, all defensible:
1. **Keep as-is.** Soft asymmetry; works. Tighten the module-level docstring on `sim.rs` to say "sim-research-internal types kept here for ergonomic co-location with the design-resource types they project from" so future cold-readers don't have to grep for consumers.
2. **Move to cf-sim-research.** `sim.rs` (145 LOC) becomes a private module of cf-sim-research. cf-device-types stops needing a `sim` submodule, becomes purely CAD-side data + shared TOML. Cleaner architectural story.
3. **Split into cf-device-types (CAD-only) + cf-sim-types (sim-research-only).** Heavy hammer; only worth it if a third binary appears that needs the sim types.

Default lean per [[feedback-strip-the-knob-when-default-works]]: option 1 (docstring tighten only) unless a concrete need surfaces. Bank for the next sim-research arc.

### N2 — cf-device-geometry transitively pulls heavy deps into cf-device-design

`cargo tree -p cf-device-design --no-default-features` shows `cf-design`, `mesh-offset`, `mesh-repair`, `mesh-sdf`, `meshopt`, `nalgebra` in the transitive tree — all via `cf-device-geometry`. These ARE load-bearing for the cached SDF + iso extraction + cavity composition that cf-device-design consumes, so this is not a problem. But the gameplan's "CAD-only" framing is slightly looser than `cargo tree` reports: cf-device-design's direct Cargo.toml is clean, the **transitive** dep cost is real (geometry stack is ~7 crates deep).

If a future sub-arc wants a thinner CAD surface (e.g., headless TOML-validator, web-preview build, smaller binary), this is where the surgery would live: split `cf-device-geometry` into `cf-device-geometry-core` (just the cached SDF types) + `cf-device-geometry-mc` (mesh-offset + mesh-repair + MC iso extraction). Premature today; bank as a Note.

### N3 — Side-arc recon code is intentionally retained as load-bearing

84 of the 109 commits are pre-gameplan-reframe F3/F4/H2/H4/CR/SL sim-arc work (e.g., `3e9958ef` H4-2-C asymmetric bound, `59997c41` F3 LM gated activation, `cab9e2e9` → `2e54a8ee` CR.0-CR.5 sliding-intruder contact recon, `381c674f` SL.4 render). Per gameplan §6, the recon **cadence** is paused, but the **code** is retained — and the dead-code agent confirmed every one of these surfaces has a current consumer in cf-sim-research:

- F3 LM gated activation: `insertion_sim.rs:865-895`, called from `run_insertion_ramp` + `run_sliding_insertion_ramp`. Bit-equal-when-dormant per [[feedback-spec-falsified-revert-opt-in-keep-surface]].
- H4-2-C Yeoh bound: `insertion_sim.rs:714-746`, consumed unconditionally by every sim execution.
- CR.0-CR.5 contact + SL.4 render: `insertion_sim.rs:2416-2750` + `insertion_sim_ui.rs:643-900`. All wired into the active sliding-intruder path.

None of these are orphans. Worth surfacing in the PR description so reviewers don't bisect through the 84 commits looking for sim-decouple bugs.

### N4 — Phase 5 cavity-label sweep on cf-sim-research INTENTIONALLY keeps H4-2-C vocab @ `tools/cf-sim-research/src/main.rs:1289-1292`

The cf-sim-research cavity-section label still reads `(capped at {:.0} mm — H4-2-C asymmetric one-sided bound dropped the family-uniform compressive floor at MaterialField::sample_yeoh, unblocking deep-compression equilibria; cavity 3 + 5 mm user-verified 16/16 — see CANDIDATE_H4_FALSIFICATION_BOOKMARK §5.4)`. Phase 5 SHIPPED block explicitly banks this: "cf-sim-research local sites... intact — the sim-research vocab is honest in that context." Per [[feedback-autonomous-architecture]] this is the right call (the label IS honest in cf-sim-research; the same label was wrong in cf-device-design). NOT a finding — flagged only because a less-careful reader might want to "polish" it and would be wrong to do so.

---

## Compliments

### C1 — Phase 5 binary-appropriate vocab decision is textbook

cf-device-design line 878: `(capped at {:.0} mm — workshop envelope)` (binary-neutral).
cf-sim-research line 1289-1292: full H4-2-C/MaterialField/Yeoh/CANDIDATE_H4_FALSIFICATION_BOOKMARK detail (sim-research-honest).
cf-device-types line 43-59: docstring describes the 8 mm in workshop terms with a one-line POINTER to the sim-research history.

That's three different vocabularies for three different audiences from a single load-bearing constant. The Phase 5 SHIPPED block frames it as "symmetric to Phase 4's pass-2 docstring sweep on cf-sim-research, opposite direction" — pinning that pattern in [[feedback-smoke-as-user-facing-cold-read]] was right.

### C2 — Triple-anchor sentinel test pattern survived the move @ `tools/cf-device-design/src/main.rs:1728` + `tools/cf-sim-research/src/main.rs:2059` + `design/cf-device-types/src/design.rs:74`

The `cavity_inset_slider_range_zero_to_eight_mm` sentinel lives in BOTH binaries' test modules, and the `CAVITY_INSET_SLIDER_MAX_M` const it tests lives in the shared cf-device-types crate. If the const drifts, both binaries' tests fail. Genuinely robust redundancy.

### C3 — Bevy `is_changed()` footgun pattern preserved across the move

Per [[project-bevy-is-changed-footgun]], `ResMut::is_changed()` is a token, not a value-diff. The `Local<Option<CavityMeshKey>>` / `Local<Option<LayerMeshKey>>` / `Local<Option<IntruderMeshKey>>` snapshot-and-compare pattern is intact in:
- cf-device-design `main.rs` (`update_cavity_mesh` + `update_layer_meshes`)
- cf-sim-research `main.rs` (same two systems + `update_intruder_mesh`)

The Phase 4 SHIPPED block explicitly called this out: "Snapshot-and-compare `Local<>` pattern preserved per [[project-bevy-is-changed-footgun]]." Verified at the call sites.

### C4 — `CF_DEVICE_DESIGN_SPIKE_SCAN` → `CF_SIM_RESEARCH_SPIKE_SCAN` rename was a real catch

`76292f34` (Phase 4 polish) renamed the env var at 9 sites (7 consumers + 2 docstring mentions). Phase 4.3's docstring sweep had renamed the cookbook `cargo test` invocations but missed the env var name itself. Workshop devs following the cookbook would have seen `cf-sim-research` while the var still spelled `CF_DEVICE_DESIGN`. Cold-read pass-3 (post-ship) caught it. Worth keeping the post-ship-cold-read pattern per [[feedback-cold-read-review-post-ship]] + [[feedback-cold-read-two-passes-for-non-trivial-diffs]].

### C5 — Workshop loop integrity: B1 + A1 cleanly separated

`cf-cast-cli/Cargo.toml` and `cf-cast/Cargo.toml` show **zero diff** main..dev. B1 is a single commit `948bfb8d` touching cf-cast-cli derive + cf-cast piece composer + cf-design solid_layered, plus a small touch on `tools/cf-device-design/src/main.rs` (config plumbing only). The plan's "cf-cast-cli untouched by A1" claim verified by `git log` and `git diff`. The scan → cf-device-design → TOML → cf-cast-cli → mold STL handoff path is structurally untouched by A1.

### C6 — Zero orphan deps, manual grep confirmed

Every direct Cargo.toml dep in all 4 crates has ≥1 source consumer (verified by `grep` for each `use $dep::*` pattern). Phase 4's "Cargo.toml dep sweep is load-bearing manual grep, not `cargo build` exit code" posture worked — `cargo build` would not have flagged orphans masked by transitive re-exposure.

### C7 — Architecture invariants all green

- `cargo build -p cf-device-design --no-default-features`: green.
- `cargo tree -p cf-device-design`: zero `sim-soft` / `sim-ml-chassis` anywhere in the tree (the FEM solver is fully gone, not just hidden behind a flag).
- `cargo tree -p cf-sim-research`: zero `cf-device-design` (sim-research doesn't depend on the CAD binary).
- `cf-device-types` ↔ `cf-device-geometry`: one-directional dep (geometry → types), no cycle.
- All four crates compile + clippy + fmt clean per Phase 5 close.

### C8 — Cold-read polish bundles are visible in the git log

Counted in `git log main..dev`: 5dd034ec (Phase 3 polish, 6 findings) + ed1bcd1d (Phase 2.5 polish, 6 findings) + 06924d2a (Phase 2.5 polish-2, 6 findings) + 7cef74fe (Phase 1 polish, 4) + 7ea5a995 (Phase 2 polish, 4) + 4b0605b9 (Phase 3 recon polish, 11) + 38713886 (A1 plan polish) + 76292f34 (Phase 4 polish env-var). The per-phase + post-ship cold-read cadence is doing the work the [[feedback-cold-read-two-passes-for-non-trivial-diffs]] memo asks for. Pattern is visibly load-bearing — without these bundles, Phase 5 would have surfaced order-of-magnitude more polish.

---

## What I checked (audit trail)

- Foundational docs read: `PROGRAM_GAMEPLAN.md`, `SIM_DECOUPLE_REFACTOR_PLAN.md` (§0 + §4 + §9), `SIM_ARCHITECTURE_AUDIT.md` (§0-§1), `CF_CAST_MOLD_WALL_RECON.md`, `sim-research/PHASE_H_ROADMAP.md` (skim).
- `cargo tree -p {cf-device-design,cf-sim-research,cf-device-types,cf-device-geometry}` with + without `--no-default-features`.
- `cargo build -p cf-device-design --no-default-features`.
- Direct-dep grep audit: each `Cargo.toml` direct dep grep'd for `use $name::` / `extern crate $name` in the crate's `src/`.
- Inherited-vocab sweep (Explore agent + spot-check verification): cf-device-design, cf-sim-research, cf-device-types, cf-device-geometry source + Cargo.toml.
- Dead-code + side-arc residue scan (Explore agent): cf-device-design post-Phase-4-strip, cf-sim-research post-Phase-3-move, F3 LM dormancy contract, CR/SL recon consumers, H4-2-C bound consumer, cf-cap-planes call site, 8 pose helpers.
- Bevy wiring: plugin order + resource set in both binaries' `run_render_app`, `Local<Option<*MeshKey>>` snapshot pattern.
- Test counts vs Phase 5 SHIPPED claim: cf-device-design 35 + cf-sim-research 99/12 + cf-device-types 24 + cf-device-geometry 50 — all match.
- B1 isolation: `git log main..dev --oneline -- tools/cf-cast-cli/ design/cf-cast/ design/cf-design/` shows a single commit `948bfb8d`; `Cargo.toml` zero-diff.
- 12 ignored tests in cf-sim-research inventoried — all documented (fixture-needing or release-mode-needing).
- Side-arc residue: 84/109 commits are paused-recon work intentionally retained per gameplan §6; each surface confirmed to have a current consumer.

## What I did NOT check (out of scope per the prompt)

- GUI smoke (Phase 5.1 + 5.2 already ran).
- Per-commit diff replay (impossible at 109-commit scale; sampled instead).
- `cargo clippy` / `cargo fmt` re-run (Phase 5 close confirms green).
- Internal correctness of paused recon code (gameplan §6 commits to NOT touching this).
- Phase H roadmap content (B2 is doc-only).

---

End of review. Posture is catalog-only per prompt; no PR comments posted, no fixes attempted. Polish findings P1-P4 can bundle into a follow-up commit on `dev` if desired, or skip — none affect merge-readiness.
