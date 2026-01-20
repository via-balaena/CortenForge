# CortenForge Infrastructure Specification

> **Purpose**: Define the immutable constraints that enable unlimited freedom within.
> Like parametric constraints in CAD, these rules let teams scale without chaos.

> **Target**: Industrial-scale reliability. ISO 26262 / IEC 62304 / DO-178C awareness.
> Enterprise-grade foundation for physical systems: robots, vehicles, medical devices.

---

## Philosophy

```
Constraints enable freedom.
Automation prevents human error.
Provenance enables trust.
Traceability enables certification.
```

---

## The Four Pillars

### 1. Quality Gates (IMPLEMENTED)
Every line of code meets A-grade standard before merging.

### 2. Supply Chain Security (IMPLEMENTED)
Every dependency is audited, every artifact has provenance.

### 3. Traceability (PLANNED)
Every test links to a requirement, every requirement to a risk.

### 4. Automation (CONTINUOUS)
Humans review; machines enforce.

---

## Current State

### Implemented (A-Grade)

| System | Tool | Enforcement |
|--------|------|-------------|
| Formatting | rustfmt | CI blocks on diff |
| Linting | clippy pedantic+nursery | CI blocks on warning |
| Testing | cargo test | CI blocks on failure |
| Coverage | tarpaulin | CI blocks if <75% (target: 90%) |
| Documentation | rustdoc | CI blocks on warning |
| Safety | clippy unwrap_used/expect_used | CI blocks on lib code violation |
| Dependencies | cargo-deny | CI blocks on advisory/license |
| Bevy-free | cargo tree | CI blocks Layer 0 violations |

### Enhancement Roadmap

```
TIER 1: Non-Negotiable Foundation        [COMPLETE]
├── cargo-audit (CVE scanning)           [x] In quality-gate.yml
├── SBOM generation (CycloneDX)          [x] In quality-gate.yml
├── Pre-commit hooks                     [x] Auto-installed via xtask/build.rs
├── Conventional commits                 [x] Enforced by commit-msg hook
├── Signed commits/releases              [ ] Branch protection (manual)
└── cargo-semver-checks                  [x] In quality-gate.yml

TIER 2: Scale Enablers                   [PARTIAL]
├── Multi-arch CI (ARM64, WASM)          [x] In quality-gate.yml
├── Benchmark regression gates           [ ]
├── Mutation testing                     [x] In scheduled.yml (weekly)
├── API stability tracking               [x] Via cargo-semver-checks
└── Traceability infrastructure          [x] requirements/ directory exists

TIER 3: Safety-Critical Ready            [FUTURE]
├── MC/DC coverage tooling               [ ]
├── Formal verification (Kani/Prusti)    [ ]
├── Tool qualification documentation     [ ]
└── Ferrocene certified compiler         [ ]
```

---

## Tier 1: Non-Negotiable Foundation

These constraints MUST be in place before scaling the team.

### 1.1 Security Scanning

**Tool**: `cargo-audit`
**Frequency**: Every CI run
**Policy**: Block on any RUSTSEC advisory

```yaml
# .github/workflows/quality-gate.yml addition
security:
  runs-on: ubuntu-latest
  steps:
    - uses: actions/checkout@v4
    - uses: rustsec/audit-check@v1
      with:
        token: ${{ secrets.GITHUB_TOKEN }}
```

**Why**: Known vulnerabilities in dependencies are the #1 attack vector.
CVE-2024 data shows 70% of breaches involve known, patchable vulnerabilities.

### 1.2 SBOM Generation

**Tool**: `cargo-cyclonedx`
**Format**: CycloneDX 1.5 (OWASP standard)
**Output**: `sbom.json` as release artifact

```bash
cargo install cargo-cyclonedx
cargo cyclonedx --format json --output-file sbom.json
```

**Why**:
- US Executive Order 14028 mandates SBOMs for federal software
- EU Cyber Resilience Act requires SBOMs for all software
- Enables downstream vulnerability scanning

### 1.3 Pre-Commit Hooks

**Tool**: git hooks auto-installed via `xtask/build.rs`
**Installation**: Automatic on first `cargo build` of any xtask command
**Checks**:
- Format check (`cargo fmt --check`)
- Clippy check (library code only, `-D warnings`)
- Commit message validation (conventional commits)

```bash
# Hooks are auto-installed to .git/hooks/ when building xtask.
# The hooks run:
cargo fmt --all -- --check || exit 1
cargo clippy --all-features -- -D warnings || exit 1
```

**Why**: Shift left. Catch issues before they hit CI.
Saves developer time (fast local feedback) and CI resources.

### 1.4 Safety Lint Policy

**Principle**: Library code must never panic on recoverable errors or silently discard Results.

**Implementation** (two-tier approach):
1. **Workspace level** (Cargo.toml):
   - `unwrap_used = "warn"`, `expect_used = "warn"` (tests can use)
   - `let_underscore_must_use = "deny"` (prevent silent failures)
2. **Crate level** (lib.rs): `#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used))]`

**Effect**:
- **Library code**: `unwrap()` and `expect()` are **denied** (compile error)
- **Test code**: `unwrap()` and `expect()` are **warned** (allowed, follows ecosystem norms)
- **Doc examples**: Allowed (clippy doesn't check doc examples for these lints)
- **Discarded Results**: `let _ = result` is **denied** everywhere (prevents silent failures)

```rust
// In every lib.rs:
// Safety: Deny unwrap/expect in library code. Tests may use them (workspace warns).
#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used))]
```

**Silent failure prevention**:
```rust
// This is DENIED - silent failure
let _ = file.write_all(data);

// Must either handle the error or explicitly allow with justification
#[allow(clippy::let_underscore_must_use)] // String::write_fmt is infallible
let _ = write!(string, "...");
```

**Rationale** (the "Siemens/Caltech" approach):
- Library `unwrap()` → production crash → **catastrophic**
- Test `unwrap()` → test failure obscured → **annoying, not catastrophic**
- Discarded Result → silent data loss → **insidious bugs**
- The entire Rust ecosystem uses `unwrap()` in tests; fighting this creates friction
- Explicit policy is better than silent configuration

**Why not stricter?**
- Requiring `#[allow]` on every test would add 200+ annotations
- Tests are not production code - they're quality infrastructure
- Community norms matter for contributor onboarding

### 1.5 Conventional Commits

**Format**: `<type>(<scope>): <description>`
**Types**: feat, fix, refactor, test, docs, chore, perf, ci
**Enforcement**: commit-msg hook (auto-installed)

```
feat(mesh-boolean): add GPU-accelerated union operation
fix(mesh-io): handle malformed STL headers gracefully
refactor(mesh-repair): extract hole-filling into separate module
```

**Why**:
- Enables automatic changelog generation
- Semantic versioning from commit history
- Clear communication of change intent

### 1.6 Signed Commits

**Requirement**: All commits to main/develop must be signed
**Methods**: GPG key or SSH key signing
**Verification**: GitHub verified badge

```bash
# Developer setup
git config --global commit.gpgsign true
git config --global user.signingkey <KEY_ID>
```

**Branch Protection Rule**:
```
Require signed commits: ✓
```

**Why**:
- Proves commit authorship (non-repudiation)
- Required for supply chain security (SLSA Level 2+)
- Defense against commit spoofing attacks

### 1.7 Semantic Versioning Enforcement

**Tool**: `cargo-semver-checks`
**Policy**: Block PRs that introduce breaking changes without major version bump

```yaml
semver:
  runs-on: ubuntu-latest
  steps:
    - uses: actions/checkout@v4
      with:
        fetch-depth: 0
    - name: Check semver
      uses: obi1kenobi/cargo-semver-checks-action@v2
```

**Why**:
- Downstream crates depend on stable APIs
- Breaking changes must be intentional and documented
- Enables confident dependency updates

---

## Tier 2: Scale Enablers

These constraints enable team growth and cross-platform reliability.

### 2.1 Multi-Architecture CI (IMPLEMENTED)

**Targets**:
| Target | OS | Arch | Status |
|--------|-----|------|--------|
| x86_64-unknown-linux-gnu | Linux | x64 | ✓ |
| x86_64-apple-darwin | macOS | x64 | ✓ |
| x86_64-pc-windows-msvc | Windows | x64 | ✓ |
| aarch64-apple-darwin | macOS | ARM64 | ✓ test-arm64 job |
| aarch64-unknown-linux-gnu | Linux | ARM64 | - |
| wasm32-unknown-unknown | WASM | - | ✓ wasm job (Layer 0) |

**Why**:
- Apple Silicon is now majority Mac market
- ARM servers (AWS Graviton, Azure Ampere) are cost-effective
- WASM enables browser deployment (CortenForge Studio vision)

### 2.2 Benchmark Regression Detection

**Tool**: Criterion + `critcmp` or GitHub Action
**Policy**: Warn on >5% regression, block on >20%
**Storage**: Benchmark results in separate branch or artifact

```yaml
benchmarks:
  runs-on: ubuntu-latest
  steps:
    - uses: actions/checkout@v4
    - name: Run benchmarks
      run: cargo bench --all-features -- --save-baseline pr
    - name: Compare to main
      run: |
        git fetch origin main
        cargo bench --all-features -- --baseline main --compare
```

**Why**:
- Performance is a feature for real-time systems
- Regressions are often invisible without measurement
- Catches accidental algorithmic complexity increases

### 2.3 Mutation Testing

**Tool**: `cargo-mutants`
**Frequency**: Weekly or on-demand (expensive)
**Target**: <30% mutation survival rate

```bash
cargo install cargo-mutants
cargo mutants --package mesh-repair -- --release
```

**Why**:
- Coverage ≠ test quality
- Mutation testing proves tests actually catch bugs
- "If I break this code, will a test fail?"

### 2.4 Traceability Infrastructure

**Structure**:
```
requirements/
├── REQ-MESH-001.yaml    # Mesh topology requirements
├── REQ-MESH-002.yaml    # Mesh I/O requirements
├── REQ-SAFETY-001.yaml  # Safety requirements
└── traceability.toml    # Links to tests
```

**Requirement Format**:
```yaml
# requirements/REQ-MESH-001.yaml
id: REQ-MESH-001
title: Mesh Validation
description: |
  The system shall detect non-manifold meshes with:
  - Open edges
  - Self-intersections
  - Degenerate triangles
priority: HIGH
verification:
  - test: mesh-repair::tests::test_detect_non_manifold
  - test: mesh-repair::tests::test_detect_self_intersection
traces_to:
  - RISK-001  # Non-manifold mesh causes print failure
```

**Test Annotation**:
```rust
/// Verifies: REQ-MESH-001
#[test]
fn test_detect_non_manifold() {
    // ...
}
```

**Why**:
- ISO 26262 / IEC 62304 / DO-178C all require bidirectional traceability
- Enables impact analysis ("what tests cover this requirement?")
- Proves completeness ("are all requirements tested?")

---

## Tier 3: Safety-Critical Ready

These prepare for formal certification (ASIL D, Class C, DAL A).

### 3.1 MC/DC Coverage

**Requirement**: Modified Condition/Decision Coverage
**Tool**: LLVM coverage with MC/DC support (experimental) or commercial tools
**When**: Targeting automotive (ISO 26262 ASIL D) or aerospace (DO-178C Level A)

**What it means**:
```rust
if a && b {  // Need tests where:
    // ...   // 1. a=true, b=true (decision true)
}            // 2. a=false, b=true (a independently affects decision)
             // 3. a=true, b=false (b independently affects decision)
```

### 3.2 Formal Verification

**Tools**:
- **Kani**: Model checker for Rust (Amazon-developed)
- **Prusti**: Verification framework (ETH Zurich)

**Targets**: Critical algorithms only
- mesh-boolean intersection detection
- mesh-repair topology operations
- sensor-fusion transforms

**Why**: Proves absence of bugs, not just presence of tests.

### 3.3 Certified Compiler

**Tool**: Ferrocene (ISO 26262 / IEC 61508 qualified)
**When**: Deploying to safety-critical hardware
**Cost**: Commercial license required

---

## Enforcement Summary

### Pre-Commit (Local)
```
┌─────────────────────────────────────────┐
│ Developer Machine                        │
├─────────────────────────────────────────┤
│ git commit                               │
│   ├── pre-commit hook (auto-installed)   │
│   │   ├── cargo fmt --check              │
│   │   └── cargo clippy -D warnings       │
│   └── commit-msg hook (auto-installed)   │
│       └── conventional commit format     │
└─────────────────────────────────────────┘
```

### CI Pipeline (Remote)
```
┌─────────────────────────────────────────┐
│ GitHub Actions                           │
├─────────────────────────────────────────┤
│ Push / PR                                │
│   ├── format (rustfmt)                   │
│   ├── lint (clippy)                      │
│   ├── test (3 platforms)                 │
│   ├── coverage (tarpaulin, ≥75%)         │
│   ├── docs (rustdoc)                     │
│   ├── safety (clippy unwrap_used deny)   │
│   ├── security (cargo-audit)             │
│   ├── dependencies (cargo-deny)          │
│   ├── bevy-free (Layer 0 check)          │
│   ├── semver (cargo-semver-checks)       │
│   ├── sbom (cargo-cyclonedx)             │
│   ├── arm64 (Apple Silicon)              │
│   └── wasm (Layer 0 compatibility)       │
│                                          │
│ Merge to main                            │
│   ├── All above pass                     │
│   ├── Signed commit verified             │
│   └── Required reviewers approved        │
│                                          │
│ Release tag                              │
│   ├── SBOM attached                      │
│   ├── Signed release                     │
│   └── Changelog generated                │
└─────────────────────────────────────────┘
```

### Periodic Jobs
```
┌─────────────────────────────────────────┐
│ Scheduled                                │
├─────────────────────────────────────────┤
│ Weekly                                   │
│   ├── cargo-audit (fresh advisories)     │
│   ├── cargo-mutants (mutation testing)   │
│   └── dependency-review (updates)        │
│                                          │
│ Monthly                                  │
│   └── benchmark regression vs baseline   │
└─────────────────────────────────────────┘
```

---

## Metrics Dashboard

Track these metrics for health visibility:

| Metric | Target | Current | Trend |
|--------|--------|---------|-------|
| Test Coverage | ≥75% (target: 90%) | ~77% | → |
| Clippy Warnings | 0 | 0 | → |
| Doc Warnings | 0 | 0 | → |
| Security Advisories | 0 | ? | - |
| Mutation Survival | <30% | ? | - |
| A-Grade Crates | 33 | 33 | → |
| Build Time (clean) | <10min | ? | - |
| Build Time (cached) | <2min | ? | - |

---

## Implementation Priority

### Completed
- [x] `cargo-audit` in CI
- [x] SBOM generation in CI
- [x] `cargo-semver-checks` in CI
- [x] ARM64 CI target (macOS)
- [x] WASM CI target (Layer 0 crates)
- [x] Mutation testing (weekly scheduled)
- [x] Traceability infrastructure (requirements/ directory)
- [x] Pre-commit hooks (auto-installed via xtask/build.rs)
- [x] Conventional commit enforcement (commit-msg hook)

### Remaining (As Needed)
- [ ] Signed commits (GitHub branch protection)
- [ ] Benchmark regression detection
- [ ] Linux ARM64 CI target
- [ ] Formal verification for critical paths
- [ ] Ferrocene evaluation for safety-critical

---

## References

- [SLSA Framework](https://slsa.dev/) - Supply chain security levels
- [OpenSSF Scorecard](https://securityscorecards.dev/) - Security health metrics
- [Ferrocene](https://ferrocene.dev/) - Safety-certified Rust compiler
- [ISO 26262](https://www.iso.org/standard/68383.html) - Automotive functional safety
- [IEC 62304](https://www.iso.org/standard/38421.html) - Medical device software
- [DO-178C](https://www.rtca.org/) - Aerospace software certification

---

*Last updated: 2026-01-18*
*Version: 1.1.0 - Updated to reflect implemented CI infrastructure*
