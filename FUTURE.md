# Future Enhancements

Ideas and planned improvements that are not yet implemented.

---

## Coverage Tracking (Rust-native)

Build a simple coverage diff tool in xtask to replace external services like Codecov.

**What it would do:**
1. Parse tarpaulin JSON output
2. Compare to previous run (stored in a file or git notes)
3. Print a diff showing what changed

**Why:**
- Pure Rust + shell, no external dependencies
- No third-party service accounts needed
- Full control over the output format

**Implementation ideas:**
- `cargo xtask coverage-diff` command
- Store baseline in `.coverage-baseline.json` or git notes
- Output like: `mesh-types: 77% -> 79% (+2%)`

---

## Benchmark Regression Detection

Automatically detect performance regressions in PRs.

**What it would do:**
1. Run criterion benchmarks
2. Compare against baseline (stored in git or artifacts)
3. Warn on >5% regression, block on >20%

**Status:** Infrastructure exists in `scheduled.yml` (weekly baseline), but no PR comparison yet.

---

## Formal Verification for Critical Paths

Use Kani or Prusti for proving absence of bugs in critical algorithms.

**Targets:**
- mesh-boolean intersection detection
- mesh-repair topology operations
- sensor-fusion transforms

**Status:** Future consideration for safety-critical deployments.

---

## Linux ARM64 CI Target

Add `aarch64-unknown-linux-gnu` to CI matrix for ARM server compatibility.

**Why:** AWS Graviton, Azure Ampere are cost-effective ARM servers.

**Status:** macOS ARM64 is tested; Linux ARM64 not yet.

---
