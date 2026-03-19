# V1.0 Roadmap

> What stands between 0.7.0 and a clean, shippable 1.0.

**Goal:** Every item here is either a doc fix, a test gap, or a housekeeping
task. The code architecture and scope are already v1.0-quality. This list is
about making the packaging match.

---

## 1. Refresh VISION.md

VISION.md still references removed domains (ml/, routing/, sensor/) as active
architecture. The Layer 0 diagram lists `sensor/*`, `ml/*`, `routing/*`. The
"Domain Coverage" table includes Sensors, ML Pipeline, and Routing as "Build"
items. The "Behemoth Comparison" section references `ml-types`, `ml-dataset`,
etc.

**Work:**
- Remove all references to ml/, routing/, sensor/ domains
- Update Layer 0 diagram to show current 3-pillar structure (design, mesh, sim)
- Update Domain Coverage table to reflect what actually ships
- Update crate counts (27 crates, not 52+)
- Update "Domain Roadmap" section
- Keep the north-star tone — just make it match reality

**Size:** ~1 hour

---

## 2. Clean up sim/docs/todo/

18 `future_work_*.md` files + 15 spec fleshout directories + a `POST_V1_ROADMAP.md`
+ an `index.md`. A newcomer opening this directory sees planning artifacts that
signal "in progress," not "finished product."

**Work:**
- Consolidate the 18 `future_work_*.md` files into `POST_V1_ROADMAP.md` (or a
  single `FUTURE_WORK.md`). One file, organized by theme, not chronologically
- Move completed spec fleshouts (phases 1–6) into `sim/docs/todo/archived/`
- Keep active/future phases (7–13) as-is
- Update `index.md` to reflect the new structure

**Size:** ~2 hours

---

## 3. Refresh stale archive docs

`docs/archive/COMPLETION_LOG.md` still lists 52 crates. Only 27 remain after
the cleanup. `docs/FUTURE.md` references `sensor-fusion` (removed).

**Work:**
- Update COMPLETION_LOG.md crate list to match current workspace, or add a
  note at the top explaining the cleanup delta
- Remove `sensor-fusion` reference from FUTURE.md
- Quick scan of all docs/archive/ files for other stale references

**Size:** ~30 min

---

## 4. cf-design test hardening

cf-design has 23.6K lines across 31 files. It has inline `#[cfg(test)]` modules
in many files, which is a start. But for a crate that is the differentiating
technology, test coverage should match the sim domain's standard.

**Work:**
- Audit current test coverage (`cargo tarpaulin -p cf-design`)
- Identify critical paths with no coverage: dual contouring, adaptive DC,
  mechanism builder, MJCF export, STL export, parameter gradients
- Add integration tests for the design-to-mesh pipeline (solid → mesh → validate)
- Add integration tests for mechanism-to-MJCF pipeline
- Target: ≥75% line coverage (matching the A-grade standard)

**Size:** ~1–2 sessions

---

## 5. README polish

The README is solid but has a minor inconsistency: it says "21 library crates"
in the Crate Overview section. Verify this matches the actual count after cleanup.
Also, cf-design is listed under "What's next" but Phases 1–4 are already
implemented.

**Work:**
- Verify and update crate count
- Move cf-design from "What's next" to "What's built" (Phases 1–4 are done)
- Update cf-design description to reflect current state vs. planned Phase 5

**Size:** ~15 min

---

## 6. Version bump to 1.0.0

Once items 1–5 are done, bump the workspace version.

**Work:**
- Update `version` in root Cargo.toml from `0.7.0` to `1.0.0`
- Update any version references in docs
- Tag the release

**Size:** ~5 min (after everything else is done)

---

## Non-goals for 1.0

These are good ideas but explicitly out of scope for the 1.0 cut:

- **cf-design Phase 5 (Differentiable Design)** — this is new feature work,
  not a 1.0 prerequisite. Ship 1.0 with Phases 1–4 stable, Phase 5 comes in 1.1
- **PyO3 bindings** — future work
- **WASM examples** — nice to have, not blocking
- **Consolidating sim/docs/ spec fleshouts into fewer files** — the archived
  specs have historical value; moving them to archive/ is sufficient
- **coverage-diff tooling** (docs/FUTURE.md) — enhancement, not 1.0
- **B-Rep upgrade path** — explicitly future
