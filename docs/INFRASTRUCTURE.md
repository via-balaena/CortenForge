# CortenForge Infrastructure Specification

> **Purpose**: Define the immutable constraints that enable unlimited freedom within.
> Like parametric constraints in CAD, these rules let teams scale without chaos.

> **Target**: Industrial-scale reliability. ISO 26262 / IEC 62304 / DO-178C awareness.
> Enterprise-grade foundation for mechatronics, simulation, and physical systems.

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
| Coverage | `cargo xtask grade` (llvm-cov) | ⚠ **Measured weekly, not per-PR.** `grade-all` still runs `--skip-coverage` on every PR shard, so coverage does not block a merge. `scheduled.yml`'s `coverage` job runs the same grader WITHOUT that flag, sharded 3 ways — one instrument, per-crate thresholds, reproducible locally by `cargo xtask grade <crate>`. It replaced a `cargo tarpaulin --workspace --fail-under 75` job that was a workspace *aggregate* and had never produced a number (compile failure, red every run since 2026-06-28). Honest tradeoff: a coverage regression is caught within a week of landing, not before. |
| Documentation | rustdoc | CI blocks on warning |
| Safety | clippy unwrap_used/expect_used | CI blocks on lib code violation |
| Dependencies | cargo-deny | CI blocks on advisory/license |
| Bevy-free | cargo tree | CI blocks Layer 0 violations |

### Enhancement Roadmap

```
TIER 1: Non-Negotiable Foundation        [COMPLETE]
├── cargo-audit (CVE scanning)           [x] In quality-gate.yml
├── SBOM generation (CycloneDX)          [x] In quality-gate.yml
├── Pre-commit hooks                     [x] Source xtask/hooks/, auto-installed
├── Conventional commits                 [x] Enforced by commit-msg hook
├── Signed commits/releases              [ ] Branch protection (manual)
└── cargo-semver-checks                  [x] In quality-gate.yml

TIER 2: Scale Enablers                   [PARTIAL]
├── Multi-arch CI (ARM64, WASM)          [x] In quality-gate.yml
├── Benchmark regression gates           [ ]
├── Mutation testing                     [x] In scheduled.yml (weekly)
├── API stability tracking               [x] Via cargo-semver-checks
└── Traceability infrastructure          [ ] (requirements/ removed — rebuild when needed)

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

**Source of truth**: `xtask/hooks/pre-commit` and `xtask/hooks/commit-msg` — the
hook scripts themselves, tracked and pinned to LF in `.gitattributes`. ⚠ To change
what a hook DOES, edit those files. Do not add hook text to `xtask/build.rs`: it
holds an `include_str!` of them, and a second copy of the text is exactly the drift
that left the scan/mesh guard uninstalled on most checkouts (#833).

**Installers**: two, both reading the same source and pairing each text with its
filename through the single `hook_install::HOOKS` table —
- `xtask/build.rs` — automatic on first `cargo build` of any xtask command. Replaces
  a hook that is ours and out of date, repairs a missing executable bit, and leaves a
  foreign hook alone. Every outcome except "already ours and current" is announced
  with a `cargo:warning`, because a guard that is not armed must never be silent.
  Two silences are deliberate: it does nothing under `CI`/`GITHUB_ACTIONS`, and it
  stays quiet when there is no `.git` at all (a vendored copy, a tarball). ⚠ The
  first bites locally if you export `CI=1` in your own shell — the build then
  installs nothing and says nothing. `cargo xtask setup` deliberately does NOT
  honour those variables, so it is the way out of that state.
- `cargo xtask setup` — the one-command heal, and `cargo xtask uninstall` its
  inverse. Both make the SAME ownership check `build.rs` makes: a hook that is not
  ours is neither overwritten nor deleted, and they say so. They differ from
  `build.rs` in no way at all: an up-to-date hook has its executable bit repaired
  IN PLACE (`chmod`), never rewritten. ⚠ Rewriting looked equivalent and was not —
  the atomic write renames over the path, so it replaced a symlinked hook with a
  regular file, and rewrote the file (new inode) on every single run. ⚠ It does NOT
  stop a tracked hook at 644 becoming 755 — that still happens, and must, because git
  ignores a hook without the executable bit.

  ⚠ This used to be the opposite: `setup` wrote both hooks unconditionally and
  `uninstall` deleted by filename, so the two installers reached opposite verdicts on
  the same file and the untested one destroyed data. Resolving the hooks directory
  properly is what forced the fix — run from a linked worktree, these commands reach
  the MAIN checkout's shared hooks, so the clobber was no longer confined to whoever
  typed the command.

Both resolve the directory by ASKING git (`rev-parse --show-toplevel
--git-common-dir --git-path hooks`) rather than joining `.git/hooks` onto the repo
root — that join is silently wrong under `core.hooksPath`, in a linked worktree
(whose `.git` is a *file*, so the path does not exist), and with
`--separate-git-dir`. In each case the old code wrote a file git never reads, and
reported success.

Asking git costs a property the naive join had for free: the join could never leave
the checkout. Three checks restore it — git's `--show-toplevel` must be the directory
we asked about (git finds a repo by walking UP, so a vendored copy resolves to the
ANCESTOR repo); the hooks directory must lie inside the common git dir or the working
tree; and `GIT_DIR`/`GIT_WORK_TREE` are cleared before asking, because with `GIT_DIR`
set git skips discovery and answers the toplevel question with our own path while
answering the others for a different repository.

If git cannot be asked at all, resolution falls back to `<root>/.git/hooks` when that
`.git` is a real directory — installing nothing would be a regression for a container
with no `git` binary or a repo tripping `dubious ownership`. ⚠ The fallback is the
LAST resort, not the second: git's own answer always wins when there is one. The one
case the fallback cannot see is a `core.hooksPath` on a machine where git cannot run
at all — nothing can, and it is the only remaining way to be told "Installed" about a
file git does not read.

The ask deliberately passes NO `--path-format=absolute`. Asking for it made git's
answer easier and the code harder: a git older than 2.31 (Ubuntu 20.04 ships 2.25)
does not know the flag, echoes it, and exits 0 — which forced a second ask, a second
parser, an absolute-path rule to tell the two apart, and a documented order between
their answers. Every option now passed predates git 2.5. Measured across 35 repository
shapes, nine of them built specifically to break the equivalence: one ask and two asks
returned the identical verdict every time, and under a pre-2.5 shim the one-ask design
is strictly better — three shapes that resolved to a SILENT refusal now resolve
correctly or say why.

⚠ Because `GIT_DIR`/`GIT_WORK_TREE` are cleared, a working tree checked out purely
through those variables (a bare repo plus `GIT_WORK_TREE`) can no longer be resolved.
That trade is deliberate — with both set, git still answers the ownership question
with our own directory while answering the rest for the other repository — and the
build says so by name rather than falling silent.

⚠ A `core.hooksPath` whose directory name begins with `-` now resolves correctly —
but git itself cannot execute a hook out of it (`/bin/sh: -/: invalid option`). It
fails CLOSED, refusing the commit, so nothing is lost silently; the setting is simply
unusable, for reasons outside this code.

Relative answers from an old git are NORMALISED, not merely joined: `..` and symlinks
are resolved the way git resolves them. Without that, `core.hooksPath =
../shared-hooks` produced `<repo>/../shared-hooks`, which `starts_with` accepts
component-wise — containment defeated, on exactly the git version the second ask
serves.

**Checks**:
- **Scan/mesh guard** — refuses staged `*.stl` / `*.obj` / `*.ply` / `*.3mf` /
  `*.mtl` / `*.step` / `*.stp` (see 1.3.1 below). Runs first: it is a safety rule,
  not a quality one.
- Format check (`cargo fmt --check`)
- Clippy check — **`--all-targets`** (lib, bins, tests, benches, examples) on the
  crates with staged changes, `-D warnings`
- Commit message validation (conventional commits). `Merge`/`Revert` subjects and
  the `fixup!`/`squash!`/`amend!` autosquash markers are allowed — rejecting those
  forced `--no-verify`, which drops the scan/mesh guard too.

```bash
# Hooks are auto-installed when building xtask, into the directory git reports.
# The hooks run:
cargo fmt --all -- --check || exit 1
# ...then clippy, scoped to the crates owning the staged files (not the workspace):
cargo clippy -p <changed> --all-targets --all-features -- -D warnings || exit 1
```

⚠ A staged file that belongs to no crate lints nothing. The walk now examines every
directory up to and INCLUDING the repository root, so a repo whose root is a crate has
that crate linted — it used to stop one level short, and the message blamed a "virtual
manifest", which was the wrong cause: a root manifest *with* a `[package]` behaved
identically. For CortenForge the root manifest genuinely is a virtual workspace, so a
root-only `Cargo.toml` change (including `[workspace.lints]`) still lints nothing.
That is a real gap: linting the whole workspace is what pre-commit speed rules out.

**Why**: Shift left. Catch issues before they hit CI.
Saves developer time (fast local feedback) and CI resources.

#### 1.3.1 No scan or mesh binaries in this repository — hard rule

**This repository is PUBLIC, and some of the casting pipeline's inputs are
anatomical scans of a real person.** They must never be committed here.

Where the data lives instead: `~/scans/`, reached by `CF_CAST_ITER1_DIR` (see
`design/cf-cast/tests/iter1_gate.rs`, which is `#[ignore]`d precisely so the standard
suite needs no out-of-repo fixture) or regenerated by `cf-cast`. Nothing in this repo
has ever tracked a mesh — `git ls-files ':(icase)*.stl' ':(icase)*.obj'
':(icase)*.ply' ':(icase)*.3mf' ':(icase)*.mtl' ':(icase)*.step' ':(icase)*.stp'`
returns zero, in the working tree and in all history (re-verified 2026-08-27).

That was **habit, not a rule**, until 2026-07-29: `git add` was observed accepting a
29 MB anatomical scan without complaint. A pushed blob is in the public history
permanently and cannot be meaningfully recalled. So it is enforced in two layers,
because the cost of a single miss is unrecoverable:

| layer | catches | cannot catch |
|---|---|---|
| `.gitignore` blanket `*.stl` `*.obj` `*.ply` `*.3mf` `*.mtl` `*.step` `*.stp` | `git add`, `git add .` | `git add -f`; and an UPPERCASE `PART.STL` wherever `core.ignorecase=false` (Linux, CI) — these patterns are case-sensitive, unlike layer 2's |
| pre-commit guard (`xtask/hooks/pre-commit`) | `git add -f`, anything already staged, any case (`:(icase)`) | a commit made with `--no-verify`; with `CF_ALLOW_MESH=1` **already exported**, a commit made while git cannot read the index (see below); and **any extension not in the list** |

⚠ **Both layers are ALLOWLISTS, and that is the largest hole.** They cover the seven
extensions above because those are what `mesh-io` reads and writes. A scan exported
as `.glb`, `.gltf`, `.off`, `.pcd`, `.xyz`, `.e57` or `.fbx` passes both layers
untouched — verified by staging each. `.step`/`.stp` were in neither layer until
2026-08-27 despite `MeshFormat::Step` existing all along, which is the shape of the
mistake to expect again: **the guard tracks the formats we support, and it lags them.**
When `mesh-io` learns a format, both layers and their test must learn it in the same
commit.

Neither layer can untrack anything: zero meshes are tracked, and `.gitignore` never
applies to files already in the index. The `sim/L0/tests/assets/mujoco_menagerie`
assets are a **submodule** — its contents belong to its own repository and are
unaffected.

⚠ `CF_ALLOW_MESH=1` has a second effect, added deliberately and worth knowing. If
git cannot read the staged file list at all, the guard cannot run; without the
override the hook REFUSES the commit, and with it the guard is skipped entirely and
says so. That is the lesser evil — the alternative is `git commit --no-verify`, which
disables the commit-message hook as well — but it means someone who exports
`CF_ALLOW_MESH=1` permanently has turned the guard into best-effort. Set it per
command, not in your shell profile.

Escape hatch, for a genuinely non-personal fixture mesh:

```bash
CF_ALLOW_MESH=1 git commit ...
```

Use it deliberately and never for anything derived from a person.

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
feat(mesh-repair): add hole-filling edge case detection
fix(mesh-io): handle malformed STL headers gracefully
refactor(mesh-sdf): extract grid interpolation into separate module
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

**Status**: The `requirements/` directory was removed (single file, no code consumers).
Traceability infrastructure will be rebuilt when the project scales to need it.

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
- mesh-repair topology operations
- sim-core critical algorithms

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
│   ├── (NO coverage — --skip-coverage;    │
│   │    measured weekly, see below)       │
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
│   ├── xtask grade-all (per-crate         │
│   │    COVERAGE — 3 shards, no           │
│   │    --skip-coverage)                  │
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
| Test Coverage | ≥75% **per crate** | see weekly `Coverage` job | → |
| Clippy Warnings | 0 | 0 | → |
| Doc Warnings | 0 | 0 | → |
| Security Advisories | 0 | ? | - |
| Mutation Survival | <30% | ? | - |
| A-Grade Crates | 20 | 20 | → |
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
- [ ] Traceability infrastructure (requirements/ removed — rebuild when needed)
- [x] Pre-commit hooks (source xtask/hooks/, auto-installed)
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

*Last updated: 2026-03-19*
*Version: 1.4.0 - Post-cleanup: 20 library crates (10 mesh + 3 design + 7 sim)*
