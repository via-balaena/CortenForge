# CortenForge architecture map

A newcomer's map of *where things live and which way dependencies point*. It is a
navigational aid, not a rulebook — the **rules** (and their enforcement) live in
[`docs/STANDARDS.md`](docs/STANDARDS.md); the **mission** lives in [`MISSION.md`](MISSION.md).

## The one invariant

**Dependencies point down the tiers, never up.** Every library crate declares a tier in
its `Cargo.toml`:

```toml
[package.metadata.cortenforge]
tier = "L0"   # L0 | L0-io | L0-integration | L1 | App
```

`cargo xtask grade` Criterion 6 (*Layer Integrity*) enforces this **hard** — an `F`, not a
warning — checked transitively across default / no-default / all-features graphs. The two
load-bearing rules:

- **App-sink rule:** no SDK-tier crate (L0/L0-io/L0-integration/L1) may depend on an
  `App`-tier crate. Apps consume the SDK; the SDK never reaches back into an app.
- **L0 is Bevy-free:** the compute spine carries no rendering dependency, so it builds
  headless (CI, servers, WASM). Presentation crates are a named, tiered-up exemption.

This *is* the "spine + plugins" architecture, enforced by construction. See
[`docs/STANDARDS.md`](docs/STANDARDS.md) §6 for the exact banned-prefix + dep-count rules.

## The layers

```text
                 apps / tools            cf-studio*, cf-spine-studio, cf-sim-research,
                 (tier = App or           cf-device-design, cf-scan-prep …
                  named exemption)        — consume the SDK; may depend on anything below
                        │  depends on ↓
   ────────────────────────────────────────────────────────────────────────────
                 L1  coupling             sim-coupling (keystone: one tape across
                 (cross-engine)           rigid+soft), cf-fsu-model, sim-bevy*
                        │  ↓
                 L0  the compute spine    sim-core (rigid), sim-soft (FEM), sim-mjcf,
                 (strictest; Bevy-free;   sim-ml-chassis, sim-opt, sim-rl, mesh-*,
                  WASM-clean)             cf-geometry, cf-design, cf-spatial …
```

- **Keystone: `cf-geometry`** — zero internal dependencies, ~19 library crates depend on
  it. The true root of the graph. `sim-types` is a second dependency-free root.
- **Facades are thin re-export shells** (~130–150 LOC each): `mesh` (the mesh stack),
  `sim` (the sim stack), `cortenforge` (the top-level umbrella). They aggregate; they
  hold no logic.
- **Big-but-coherent engines, not god-crates:** `sim-core` (~71k LOC) is the MuJoCo-class
  rigid-dynamics rebuild — one solver universe; `sim-mjcf` (~32k) is the MJCF parse/emit.
  Large because their domain is large, not because they do unrelated things.
- **Presentation is quarantined:** Bevy lives only in the presentation crates — `cf-viewer`,
  `cf-bevy-common`, the `*-bevy` and `*-gui` crates, `cf-mesh-paint`. The spine never
  imports them.
- **255 example crates** under `examples/**` (plus a handful of bench/test crates) are leaf
  *consumers* — allowed to depend on anything; out of scope for the tier rules.

## Caveat: directory ≠ tier

The **tier metadata is authoritative, not the folder.** In particular `design/` is not a
layer — it mixes true spine (`cf-geometry`, `cf-design`, `cf-spatial`) with domain plugins
(`cf-cast`, `cf-routing`, `cf-fsu-geometry`, `cf-device-*`). The dependency *direction* is
still correct throughout; only the folder name is ambiguous. When in doubt, read the crate's
`tier`, not its path.

## Audit snapshot (2026-07-12)

A dependency-graph audit of the 62 library crates, recorded here as a point-in-time
measurement (numbers drift; the invariants above do not):

- **The library graph is an acyclic DAG.** No real cycles. (One *apparent* cycle,
  `sim-coupling ↔ cf-fsu-model`, is a **dev-dependency** — coupling's tests use the FSU
  model as a fixture — which does not affect the build graph. Benign.)
- **No ungoverned presentation leaks.** `cf-device-geometry` carries Bevy, but as a
  **deliberate, documented exemption** in the grader (a shared device-side compute +
  rendering crate), not an accidental leak.
- **The spine is clean:** the core compute crates depend on neither Bevy nor a domain
  crate; every edge points down.

Conclusion: the structure is sound. The recurring "is this getting too big?" question is
**bigness, not tangle** — and bigness on a coherent spine is not a defect. Cleanup here is
consumer-gated, never a foreground sprint.
