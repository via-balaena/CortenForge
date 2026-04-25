# L0 Current State — Architectural Recon

**Date:** 2026-04-25
**Trigger:** Phase 0 PR #216 surfaced a 38-min CI test-matrix runtime on free-tier GH runners; investigation found multiple independent dep-graph leaks.
**Purpose:** Ground-truth document. No recommendations — those come after we agree on the picture.

---

## TL;DR

The "L0" label covers 13 crates that range from 20 to 165 transitive deps. Treating them as one architectural layer is not coherent. The grader and STANDARDS.md describe an L0 invariant ("Bevy-free, WASM-buildable, pure compute substrate") that's enforced in narrowed form on the default-features graph only — every leak found in this recon was permitted because the actual checks don't constrain `--all-features`, dev-dep weight, transitive bloat, or feature-gated heavy chains. The 38-min CI is a downstream symptom.

---

## 1. The 13 L0 crates by weight class

Source: `cargo tree -p <crate> -e normal --no-default-features --prefix=none --format '{p}'` (and variants). Exact output: `/tmp/cortenforge-recon/dep_summary.txt`.

| Crate | Rel-Def | Rel-All | Test-All | Δ-test | Class |
|---|---:|---:|---:|---:|---|
| `sim-types` | 22 | 25 | 25 | 0 | LIGHT |
| `sim-simd` | 20 | 20 | 82 | +62 | LIGHT (rel) / dev-poisoned |
| `sim-core` | 30 | 39 | 77 | +38 | LIGHT (rel) / dev-poisoned |
| `sim-thermostat` | 39 | 39 | 165 | +126 | LIGHT (rel) / dev-poisoned (heavy) |
| `sim-gpu` | 78 | 78 | 78 | 0 | MEDIUM (wgpu by design) |
| `sim-mjcf` | 150 | 157 | 161 | +4 | **HEAVY** (load-bearing) |
| `sim-urdf` | 151 | 154 | 158 | +4 | **HEAVY** (inherits mjcf) |
| `sim-conformance-tests` | 152 | 152 | 157 | +5 | **HEAVY** (test crate at L0??) |
| `sim-ml-chassis` | 46 | 95 | 225 | +130 | LIGHT (rel-def) / bevy-leak (all-feat) / dev-poisoned (heavy) |
| `sim-rl` | 162 | 162 | 162 | 0 | **HEAVY** (inherits mjcf) |
| `sim-opt` | 48 | 48 | 164 | +116 | LIGHT (rel) / dev-poisoned (heavy) |
| `sim-soft` | 78 | 120 | 120 | 0 | MEDIUM (gpu-probe by design) |
| `sim-therm-env` | 163 | 163 | 165 | +2 | **HEAVY** (inherits mjcf) |

**Two weight classes are in play, sharing the L0 label:**
- LIGHT (≤80 release deps): `sim-types`, `sim-simd`, `sim-core`, `sim-thermostat`, `sim-gpu`, `sim-soft`, `sim-ml-chassis` (default), `sim-opt`
- HEAVY (>120 release deps): `sim-mjcf`, `sim-urdf`, `sim-conformance-tests`, `sim-rl`, `sim-therm-env`

The 6x range across the layer is the root signal that "L0" doesn't mean one thing.

---

## 2. The contagion vector: sim-mjcf

`sim-mjcf` directly depends on `image` v0.25.10 (used at `builder/mesh.rs:295` for texture loading, `builder/mod.rs:1134` for HField terrain — both load-bearing for MuJoCo MJCF spec compliance) and on `mesh-io` (which directly depends on `zip` for 3MF format support at `threemf.rs:48`).

Every L0 crate that depends on `sim-mjcf` inherits the `image → ravif/rav1e/tiff/exr/image-webp` chain plus `zip → zstd → zstd-sys → cc`. That includes:

- `sim-rl` (release, for stock task factories per SD-5)
- `sim-therm-env` (release, for ThermCircuitEnv MJCF model build)
- `sim-urdf` (release)
- `sim-conformance-tests` (release)
- `sim-mjcf` itself (release)
- `sim-ml-chassis` (dev-dep — for `cfg(test)` model fixtures)
- `sim-thermostat` (dev-dep — same pattern)
- `sim-opt` (dev-dep — same pattern)

Result: 7 of 13 L0 crates have the image+zip chain in their release graph; 3 more have it in their test graph via dev-deps. Only 3 crates (`sim-types`, `sim-simd`, `sim-core`) avoid it entirely.

This is **load-bearing**, not careless. MuJoCo needs textures and HFields. 3MF needs zip. The architectural problem is that L0 was defined too loosely to absorb crates with legitimate-but-heavy needs, so they all share a layer label and pollute siblings.

---

## 3. Opt-in features that change the graph

| Crate | Feature | Effect | Status |
|---|---|---|---|
| `sim-ml-chassis` | `bevy = ["dep:bevy_ecs"]` | Pulls 9 bevy_* sub-crates into release graph under `--all-features` | **LEAK** — undetected by grader (Bevy-free check skips `--all-features`) |
| `sim-soft` | `gpu-probe = ["dep:wgpu", "dep:bytemuck", "dep:pollster"]` | Pulls wgpu chain into release graph under `--all-features` | **BY DESIGN** — soft-body skeleton scope §1 I-6 explicitly opt-in for GPU build-graph testing |
| `sim-mjcf` | (none beyond default) | — | — |
| `mesh-io` | (none — zip is unconditional) | All consumers pay zip cost regardless of need | Could be feature-gated; isn't |

The bevy_ecs leak is the kind of thing the grader's Bevy-free criterion is supposed to catch but doesn't because it never runs `cargo tree --all-features`.

---

## 4. Dev-dep pollution

Three dev-deps systematically poison test-compile graphs across the layer:

- **`criterion`** (benchmark harness): pulls `plotters → plotters-svg → plotters-backend → tinytemplate → ciborium → ciborium-io → ciborium-ll`. Workspace dev-dep on `sim-simd`, `sim-core`, `sim-ml-chassis`. Adds ~9 crates to test compile per consumer.
- **`sim-mjcf`** (for MJCF test fixtures): pulls the entire image+zip chain (~100+ transitive crates) into test compile. Workspace dev-dep on `sim-thermostat`, `sim-opt`, `sim-ml-chassis`. SD-5 cleaned the release-graph dep; the test-graph dep was kept because removing it requires restructuring how tests get model fixtures (cyclic-sim-rl-dev-dep workaround documented at `sim-ml-chassis/Cargo.toml:37-44`).
- **Combined effect**: `sim-thermostat` has 39 release deps and 165 test deps (Δ +126). `sim-opt` has 48 release deps and 164 test deps (Δ +116). `sim-ml-chassis` has 46 release deps (default) and 225 test deps (Δ +179 vs default-rel, +130 vs all-feat-rel).

Tests aren't asserting on this content; they're just compiling against it because the dev-dep graph entrains it.

---

## 5. Enforcement gaps

Three layers of policy: STANDARDS.md (aspirational requirements), `xtask grade` (Rust-implemented checks), CI workflow (invokes the grader + supplementary jobs). Each implements a strict subset of the previous.

### Bevy-free criterion
| Source | Says |
|---|---|
| STANDARDS.md §6 | "`cargo tree` shows no bevy or winit"; "No bevy types in public API"; "Can be compiled for `--target wasm32-unknown-unknown` without bevy" |
| `xtask grade.rs:1959-2018` | Runs `cargo tree -p <crate>` with **no `--all-features`**. Checks for `bevy*`/`winit` package-name prefixes in default graph. Doesn't check API surface. Doesn't check WASM. |
| CI WASM job | Runs `cargo check --target wasm32-unknown-unknown` on 5 of 13 L0 crates. Failures are `::warning::` not errors. |
| Reality | sim-ml-chassis ships bevy_ecs in `--all-features` graph; grader passes; STANDARDS.md violated; CI doesn't catch. |

### Dependencies criterion
| Source | Says |
|---|---|
| STANDARDS.md §5 | "Heavy dependencies are feature-gated"; "Does it pull in transitive dependencies we don't want?"; "Is there a lighter alternative?" |
| `xtask grade.rs:1868-1944` | Counts direct deps via `cargo metadata`. "Heavy" if >10 (informational). Hard-gates on Cargo.toml-line-level justification comments only. |
| Reality | Multiple L0 crates pull 150+ transitive crates with all direct deps "justified." Grade A across the board. STANDARDS.md's transitive-bloat checklist items are not enforced. |

The pattern: the doc has 20+ requirements, the grader implements maybe half in narrowed form, CI uses a strict subset of the grader, reality leaks past all of these.

---

## 6. CI architecture vs runner constraint

`.github/workflows/quality-gate.yml`:

- **Format / Dependencies / Semver / SBOM / WASM / xtask Grade** — fast-tier (≤2 min each, except xtask Grade at ~14 min). Single ubuntu runner per job.
- **Test (3-OS matrix)** — runs `cargo test --all-features -p <23 crates>` on ubuntu/macos/windows in series. Observed wall time: 8 min (when ml-chassis fails fast) to 60+ min (when it has to compile the bevy_ecs + image + zip + criterion chains for every entrained crate).
- **Quality Gate (final)** — `needs: [format, grade, test, dependencies]`. Test matrix is load-bearing.

Two structural mismatches:

1. **Redundancy with grader.** xtask grade-all already runs per-crate `cargo test` (default features). The matrix re-runs every crate's tests under `--all-features`. The marginal info from `--all-features` test execution is small relative to its cost.
2. **Designed-for-capacity-we-don't-have.** Free GH runners are 2-vCPU 7-GB. Running `cargo test --all-features` against 23 crates with the heavy chains inherited above on those is a 30-60 min job. The architecture assumes capacity rather than designing to fit the constraint. xtask grade-all uses `--skip-coverage` in CI (coverage takes 5-10 min per crate); test matrix has no equivalent escape valve.

---

## 7. What L0 means today, in practice

The intended definition (STANDARDS.md §6, ARCHITECTURE.md): "L0 = Bevy-free, WASM-buildable, pure compute substrate that can be used in CLI tools, web (WASM), servers, embedded, other game engines, Python bindings."

The de-facto definition (what the codebase actually contains): "Any sim-* crate that doesn't directly depend on Bevy under default features."

Gap symptoms visible in this recon:

- Crates with 150+ transitive deps share a layer label with crates that have 20.
- Crates that drag the entire image/zip/mesh chain into release pass the Bevy-free criterion because they don't drag Bevy specifically.
- A "test crate" (`sim-conformance-tests`) lives in L0 with 152 release deps.
- Opt-in features can pull L1-typical chains into the release graph under `--all-features` and the grader doesn't notice.
- Dev-deps can pull L1-typical chains into the test graph and nothing checks this at all.

The label "L0" exists; the architectural property it should denote does not, in any enforceable form.

---

## 8. What this recon does NOT do

- **Does not propose any refactor.** The next step is agreeing on the picture, then scoping the refactor.
- **Does not audit L1, mesh-***, or **cf-***. Mesh-io's `zip` dep was traced because it's load-bearing for L0; the rest of mesh-* and all of cf-* and L1 are out of scope for this recon.
- **Does not benchmark CI runtime under hypothetical refactors.** Numbers above are observed, not projected.
- **Does not assign blame.** Every prior decision (SD-4 narrow scope, SD-5 release-graph-only fix, sim-mjcf as L0, criterion as workspace dev-dep) was reasonable in its own context. The cumulative effect is the disease, not any individual choice.
- **Does not commit to the "L0/L1" two-tier framing being correct.** The recon surfaces that the framing has problems; whether the fix is finer-grained tiers, weight-class splits, or restructuring particular crates is a refactor question.

---

## Appendix: artifacts

- `/tmp/cortenforge-recon/dep_summary.txt` — per-crate raw cargo-tree summaries
- This document — synthesis
- Source positions cited inline (file:line)
