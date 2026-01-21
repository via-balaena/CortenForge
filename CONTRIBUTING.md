# Contributing to CortenForge

> **The Proejct Face Tattoo:** A-grade or it doesn't ship. No exceptions.

---

## Philosophy

CortenForge builds infrastructure for creating physical things—from humanoid robots to vehicles to medical devices. Our code must be as reliable as the things built with it.

We maintain **A-grade academic standards** for all code. This is the cultural foundation of the project. Every contributor—human or AI—operates under the same standards.

---

## Quick Start

```bash
# Clone the repository
git clone https://github.com/cortenforge/forge.git
cd forge

# Build - git hooks are installed automatically
cargo build

# Check if your environment is ready
cargo xtask check

# Grade a specific crate
cargo xtask grade <crate-name>

# Run the full quality suite locally (same as CI)
cargo xtask ci
```

> **Note:** Git hooks (pre-commit and commit-msg) are automatically installed
> when you first build the project. No manual setup required.

### Local CI/CD (Recommended)

**Always run local checks before pushing.** Don't wait for GitHub CI to catch issues.

```bash
# Quick check (format + clippy + tests) - run this before every push
./scripts/local-quality-check.sh quick

# Full quality gate (mirrors GitHub CI)
./scripts/local-quality-check.sh

# Individual checks
./scripts/local-quality-check.sh fmt      # Format only
./scripts/local-quality-check.sh clippy   # Clippy only
./scripts/local-quality-check.sh test     # Tests only
./scripts/local-quality-check.sh docs     # Documentation only
./scripts/local-quality-check.sh safety   # unwrap/expect check
./scripts/local-quality-check.sh deps     # Dependency policy
./scripts/local-quality-check.sh bevy     # Bevy-free (Layer 0)
./scripts/local-quality-check.sh wasm     # WASM compatibility
./scripts/local-quality-check.sh coverage # Coverage threshold
```

### Using `act` for Full GitHub Actions Emulation

If you have Docker installed, you can run the exact GitHub Actions workflows locally using [act](https://github.com/nektos/act):

```bash
# Install act (macOS)
brew install act

# Run the quality gate workflow
act -j format -j clippy -j test

# Run a specific job
act -j clippy
```

The `.actrc` file is pre-configured for this project.

### Optional: Local Coverage (Linux only)

```bash
# Install tarpaulin for local coverage checking
cargo install cargo-tarpaulin

# Check coverage for a crate
cargo tarpaulin -p mesh-types --out Html
open tarpaulin-report.html

# To match CI threshold enforcement:
cargo tarpaulin -p mesh-types --fail-under 75
```

This is optional - CI enforces ≥75% coverage (target: 90% as test coverage matures).
Note: tarpaulin only works reliably on Linux. Mac/Windows users should rely on CI for coverage.

---

## The Rhythm

```
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│     Implement → Grade → Refactor (if < A) → Complete        │
│                                                             │
│     Nothing moves forward until it's A-grade.               │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Step 1: Implement

Write your code. Follow the patterns established in existing crates.

### Step 2: Grade

```bash
cargo xtask grade <crate-name>
```

This runs automated checks and shows your current grade. If any criterion is below A, you're not done.

### Step 3: Refactor

If your grade is below A on any criterion, fix it. This is not optional. The refactor loop continues until all criteria are A.

### Step 4: Complete

```bash
cargo xtask complete <crate-name>
```

This records completion in the crate's `COMPLETION.md` and updates the project-wide `COMPLETION_LOG.md`. Requires human review for API criterion.

---

## The A-Grade Standard

| Criterion | A Standard | Automated? |
|-----------|------------|------------|
| **Test Coverage** | ≥75% line coverage (target: 90%) | Yes |
| **Documentation** | Zero doc warnings, all public items documented | Yes |
| **Clippy** | Zero warnings | Yes |
| **Safety** | Zero `unwrap()`/`expect()` in library code | Yes |
| **Dependencies** | Minimal, each justified | Partial |
| **Bevy-free** (Layer 0) | No bevy/wgpu/winit in dependency tree | Yes |
| **API Design** | Intuitive, idiomatic, follows Rust guidelines | Manual |
| **Performance** | Hot paths benchmarked | Manual |

**See [STANDARDS.md](./STANDARDS.md) for full details on each criterion.**

---

## The Quality Gate

### What CI Checks

Every push and PR runs:

```yaml
- cargo fmt --check          # Formatting is law
- cargo clippy -D warnings   # All warnings are errors
- cargo test --all-features  # Tests must pass
- cargo doc -D warnings      # Docs must build clean
- coverage ≥ 75%             # Test coverage enforced (target: 90%)
- no bevy in Layer 0         # Architecture enforced
```

### What Blocks Merging

- Any CI failure
- Any crate dropping below A-grade
- New crates without COMPLETION.md
- API changes without review

---

## Project Structure

```
forge/
├── CONTRIBUTING.md      ← You are here
├── STANDARDS.md         ← Full quality criteria
├── COMPLETION_LOG.md    ← All completed crates
├── xtask/               ← Quality enforcement tool
│
├── crates/              ← Foundation algorithms
├── mesh/                ← Mesh domain
├── geometry/            ← Curves and surfaces
├── routing/             ← 3D pathfinding
├── ml/                  ← Machine learning
├── vision/              ← Computer vision
├── sim/                 ← Simulation
└── cortenforge/         ← Bevy SDK umbrella
```

### Per-Crate Structure

Every crate has:

```
mesh-types/
├── Cargo.toml
├── README.md            ← Links to STANDARDS.md
├── COMPLETION.md        ← This crate's A-grade record
├── benches/             ← Criterion benchmarks
└── src/
    └── lib.rs           ← Module docs reference standards
```

---

## Layer 0 vs Layer 1

### Layer 0: Pure Rust (No Bevy)

These crates have **zero** game engine dependencies:

- `mesh-*` crates
- `curve-types`, `lumen-geometry`
- `cf-spatial`
- `route-*` crates
- `ml-*` crates
- `vision-core`, `sim-core`

**Enforcement:** `cargo tree -p <crate> | grep bevy` must return nothing.

### Layer 1: Bevy SDK

The `cortenforge` crate wraps Layer 0 with Bevy plugins. This is the only place Bevy appears.

---

## For AI Agents

If you are an AI agent contributing to this project:

1. **Always run `cargo xtask check` before proposing changes**
2. **Grade your work with `cargo xtask grade <crate>`**
3. **Do not mark anything complete without human review**
4. **Reference STANDARDS.md for all quality decisions**
5. **When in doubt, choose the more rigorous option**
6. **Follow the established patterns in existing crates**
7. **Maintain the A-grade standard—no shortcuts**

### The Standard Prompt

When working on CortenForge, include this context:

```
This project maintains A-grade academic standards. All code must:
- Have ≥75% test coverage (target: 90%)
- Have zero clippy/doc warnings
- Have zero unwrap/expect in library code
- Be reviewed via `cargo xtask grade <crate>`

See CONTRIBUTING.md and STANDARDS.md for full requirements.
```

---

## Code Review Process

### For Contributors

1. Run `cargo xtask grade <crate>` before requesting review
2. Ensure all automated criteria are A
3. Self-review against the API checklist in STANDARDS.md
4. Create PR with grade output in description

### For Reviewers

1. Verify `cargo xtask grade` passes
2. Review API against the manual checklist
3. Check for architectural consistency with existing crates
4. Approve only if all criteria are A

### Approval Requirements

- **New crates:** Two reviewers, including maintainer
- **Existing crates:** One reviewer
- **Critical paths:** Maintainer approval required

---

## Commit Messages

Follow conventional commits:

```
<type>(<scope>): <description>

[optional body]

[optional footer]
```

Types:
- `feat`: New feature
- `fix`: Bug fix
- `refactor`: Code change that neither fixes nor adds
- `test`: Adding tests
- `docs`: Documentation only
- `chore`: Build, CI, tooling

Examples:
```
feat(mesh-types): add VertexAttributes struct
fix(mesh-io): handle empty STL files gracefully
refactor(mesh-repair): extract weld_vertices to module
test(mesh-geodesic): add edge case tests for disconnected meshes
docs(mesh-types): add examples to all public functions
```

---

## Getting Help

- **Questions:** Open a Discussion
- **Bugs:** Open an Issue with reproduction steps
- **Features:** Open an Issue for discussion first
- **Security:** Email security@cortenforge.org (do not open public issue)

---

## License

By contributing, you agree that your contributions will be licensed under Apache-2.0.

---

## The Promise

When you contribute to CortenForge, you're not just adding code. You're maintaining a standard. The A-grade standard is what makes this project trustworthy for building real, physical things.

Every contributor before you maintained this standard. Every contributor after you will too.

**A-grade or it doesn't ship.**
