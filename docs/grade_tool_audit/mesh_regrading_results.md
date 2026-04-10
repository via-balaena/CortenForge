# Mesh Crate Re-Grading Results

> Results of running the rebuilt `cargo xtask grade` on all 9 mesh crates.
> Previously, all 9 had COMPLETION.md files created in the initial commit
> (`07f07db`) using the broken grade tool with `--skip-review`. These are
> the first independent verifications.

**Date:** 2026-04-10
**Tool version:** Post-rebuild (Steps 1-7 complete)

## Summary

All 9 mesh crates fail under the rebuilt tool. **Zero** would achieve
A-grade today. The most common failures:

- **Coverage (C1):** 8/9 fail (9.9%–40.6% actual; all previously reported as A)
- **Dependencies (C5):** 8/9 fail (unjustified Cargo.toml comments)
- **Clippy (C3):** 4/9 fail (unjustified `#[allow(clippy::` attributes)
- **Documentation (C2):** 0/9 fail (all genuinely clean)
- **Safety (C4):** 0/9 fail (all genuinely clean)
- **Bevy-free (C6):** 0/9 fail (all genuinely clean)

## Per-Crate Results

### mesh-types

| Criterion | Result | Grade |
|-----------|--------|-------|
| 1. Coverage | 9.9% | F |
| 2. Documentation | 0 warnings | A |
| 3. Clippy | 0 warnings, 1 unjustified allow | F |
| 4. Safety | 0 violations | A |
| 5. Dependencies | 4 deps, all justified | A |
| 6. Bevy-free | confirmed | A |
| **AUTOMATED** | | **F** |

### mesh-io

| Criterion | Result | Grade |
|-----------|--------|-------|
| 1. Coverage | 24.3% | F |
| 2. Documentation | 0 warnings | A |
| 3. Clippy | 0 warnings, 1 unjustified allow | F |
| 4. Safety | 0 violations | A |
| 5. Dependencies | 12 deps (heavy), unjustified | F |
| 6. Bevy-free | confirmed | A |
| **AUTOMATED** | | **F** |

### mesh-repair

| Criterion | Result | Grade |
|-----------|--------|-------|
| 1. Coverage | 40.6% | C |
| 2. Documentation | 0 warnings (missing_docs not enabled) | A |
| 3. Clippy | 0 warnings | A |
| 4. Safety | 0 violations | A |
| 5. Dependencies | 10 deps, 10 unjustified | F |
| 6. Bevy-free | confirmed | A |
| **AUTOMATED** | | **F** |

### mesh-sdf

| Criterion | Result | Grade |
|-----------|--------|-------|
| 1. Coverage | 11.5% | F |
| 2. Documentation | 0 warnings (missing_docs not enabled) | A |
| 3. Clippy | 0 warnings | A |
| 4. Safety | 0 violations | A |
| 5. Dependencies | 4 deps, 4 unjustified | F |
| 6. Bevy-free | confirmed | A |
| **AUTOMATED** | | **F** |

### mesh-offset

| Criterion | Result | Grade |
|-----------|--------|-------|
| 1. Coverage | 15.7% | F |
| 2. Documentation | 0 warnings (missing_docs not enabled) | A |
| 3. Clippy | 0 warnings | A |
| 4. Safety | 0 violations | A |
| 5. Dependencies | 5 deps, 5 unjustified | F |
| 6. Bevy-free | confirmed | A |
| **AUTOMATED** | | **F** |

### mesh-shell

| Criterion | Result | Grade |
|-----------|--------|-------|
| 1. Coverage | 17.4% | F |
| 2. Documentation | 0 warnings (missing_docs not enabled) | A |
| 3. Clippy | 0 warnings | A |
| 4. Safety | 0 violations | A |
| 5. Dependencies | 9 deps, 9 unjustified | F |
| 6. Bevy-free | confirmed | A |
| **AUTOMATED** | | **F** |

### mesh-measure

| Criterion | Result | Grade |
|-----------|--------|-------|
| 1. Coverage | 21.8% | F |
| 2. Documentation | 0 warnings | A |
| 3. Clippy | 0 warnings, 1 unjustified allow | F |
| 4. Safety | 0 violations | A |
| 5. Dependencies | 5 deps, 5 unjustified | F |
| 6. Bevy-free | confirmed | A |
| **AUTOMATED** | | **F** |

### mesh-printability

| Criterion | Result | Grade |
|-----------|--------|-------|
| 1. Coverage | 21.2% | F |
| 2. Documentation | 0 warnings (missing_docs not enabled) | A |
| 3. Clippy | 0 warnings, 1 unjustified allow | F |
| 4. Safety | 0 violations | A |
| 5. Dependencies | 5 deps, 5 unjustified | F |
| 6. Bevy-free | confirmed | A |
| **AUTOMATED** | | **F** |

### mesh-lattice

| Criterion | Result | Grade |
|-----------|--------|-------|
| 1. Coverage | 37.8% | F |
| 2. Documentation | 0 warnings (missing_docs not enabled) | A |
| 3. Clippy | 0 warnings | A |
| 4. Safety | 0 violations | A |
| 5. Dependencies | 6 deps, 2 unjustified | F |
| 6. Bevy-free | confirmed | A |
| **AUTOMATED** | | **F** |

## Analysis

The rebuilt tool reveals that the previous A-grade certifications were
hallucinated by broken gates:

- **Coverage (B2/A2):** The old tarpaulin gate either hung on macOS or
  reported workspace-wide coverage. Real per-crate coverage ranges from
  9.9% (mesh-types) to 40.6% (mesh-repair). No mesh crate reaches 75%.

- **Clippy (B1):** The old gate read stdout but clippy writes to stderr.
  4 crates have unjustified `#[allow(clippy::` attributes that were invisible.

- **Dependencies (F4/F5):** The old gate used dep-count thresholds. The
  rebuilt gate requires Cargo.toml justification comments. 8/9 crates
  have unjustified dependencies (only mesh-types is fully justified).

- **Documentation, Safety, Bevy-free:** These 3 criteria genuinely pass
  for all 9 crates. The old gates happened to produce correct results
  despite their bugs (documentation by luck, safety by the blanket
  assert exclusion not mattering for these specific crates).

## Disposition

Fixing the mesh crates is out of scope for the grade tool rebuild
(chassis ss6.2). These findings are documented for a separate mesh
crate remediation initiative.
