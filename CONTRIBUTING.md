# Contributing to CortenForge

> **The Project Face Tattoo:** A-grade or it doesn't ship. No exceptions.

---

## Philosophy

CortenForge is an open-source SDK written in pure Rust for the mechatronics and simulation space — spanning design, simulation, and manufacturing. Our code must be as reliable as the things built with it.

We maintain **A-grade academic standards** for all code. This is the cultural foundation of the project. Every contributor—human or AI—operates under the same standards.

---

## Quick Start

```bash
# Clone the repository
git clone https://github.com/via-balaena/CortenForge.git
cd CortenForge

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
# Quick check (format + clippy + tests)
cargo xtask check

# Full CI suite (same as GitHub Actions)
cargo xtask ci

# Grade a specific crate
cargo xtask grade <crate-name>
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

### Local Coverage — the only place it is checked

```bash
# The instrument the A-grade standard is defined by. Cross-platform;
# needs `rustup component add llvm-tools-preview`, no extra install.
cargo xtask grade <crate>          # criterion 1 reports the number
```

⚠ **PR CI does not measure coverage, so nothing but this command catches a
regression.** `quality-gate.yml` passes `--skip-coverage` on every shard; the
only CI-side coverage is a weekly `scheduled.yml` job, which uses a different
instrument at workspace granularity and has failed every run since 2026-06-28.
Run `xtask grade` on any crate you touch, before you push.

Do not reach for `cargo tarpaulin` to predict your grade — different
instrument, different scope, and it will not agree with the number that
governs. And it is not Linux-only: `xtask grade` runs on macOS and Windows,
and the 2026-08-16 per-crate census was taken on Apple Silicon.

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

⚠ For coverage, aim past the bar rather than at it. The measurement is not
reproducible to the line: re-measuring the same tree twice, `cf-studio-engine`
returned 605/807 and 604/807, and `cf-codesign` moved six lines. Verdicts held
in both cases, but a crate landing exactly on 75.0 % is not reliably an A.

### Step 4: Complete

```bash
cargo xtask complete <crate-name>
```

This records completion in the crate's `COMPLETION.md` and updates the project-wide `docs/archive/COMPLETION_LOG.md`. Requires human review for API criterion.

⚠ **`complete` refuses a crate whose coverage is deferred** (see report-only
below), even though the grade summary shows `A`. A completion record asserts
the bar was cleared; a deferred crate's was waived. Take it over 75 % and
remove it from `COVERAGE_REPORT_ONLY` in `xtask/src/grade.rs` first.

### `(report-only)` in a grade

A handful of crates print their coverage with `(report-only)` beside it and a
grade of `—`:

```text
║ 1. Coverage      │ 69.6% (report... │   —   │ ≥75%/≥9...     ║
```

That means **measured, but the threshold is not yet enforced for this crate** —
never "not measured". The real percentage is on the row, the grade it would
have taken is in the detail line, and the per-file triage table prints as
normal, so the work is visible. Every other criterion gates it exactly as it
gates any crate, and a failing test run is never waived.

The list is a finite to-do in `xtask/src/grade.rs`; it exists because one fix
brought 14 previously-unmeasured libraries into scope at once. Enforcing one is
a deletion from that list. A crate you add today is enforced from its first
grade — nothing puts it on the list automatically. Full rationale in
[STANDARDS.md](docs/STANDARDS.md) under Criterion 1.

---

## The A-Grade Standard

| Criterion | A Standard | Automated? |
|-----------|------------|------------|
| **Test Coverage** | ≥75% **production** line coverage (target: 90%) — `#[cfg(test)]` code is excluded from the ratio; see STANDARDS.md | Yes, but **local only** — PR CI skips it |
| **Documentation** | Zero doc warnings, all public items documented | Yes |
| **Clippy** | Zero warnings | Yes |
| **Safety** | Zero `unwrap()`/`expect()` in library code | Yes |
| **Dependencies** | Minimal, each justified | Partial |
| **Bevy-free** (Layer 0) | No bevy/wgpu/winit in dependency tree | Yes |
| **API Design** | Intuitive, idiomatic, follows Rust guidelines | Manual |
| **Performance** | Hot paths benchmarked | Manual |

**See [STANDARDS.md](./docs/STANDARDS.md) for full details on each criterion.**

---

## Long Suites Run in Parallel — Non-Negotiable

**A test suite that could run concurrently and does not is a defect, and it is
reviewable as one.** This has cost real days. The licence-gated surface ran
strictly serially — one `cargo test` at a time, `--test-threads=1` inside each —
on a 12-core machine, pegging **one core**. Measured 2026-08-09: `cf-fsu-model`
alone took **1 h 46 m to finish 12 of its 27 gates**, projecting to ~4 h for that
one crate; the whole surface is longer still. (An earlier full-surface run is
recorded at 2 h 21 m on 2026-08-07 in `design/cf-fsu-geometry/BODYPARTS3D.md`,
but the surface has grown since and that figure no longer predicts a run — do not
quote it as current.) Nothing about the workload required serial execution; it
was scheduling loss, and it recurred because nothing in the repo said not to.

The rule, for any suite or tool you add or touch:

1. **Default to concurrent.** Serial is a choice that needs a written reason at
   the call site — an actual shared resource (a fixed port, a temp path, a
   global env var, a GPU), not "it was simpler."
2. **Bound concurrency by the scarce resource, usually MEMORY, not cores.** The
   heavy Tet10 arms peak near 5 GB resident; twelve at once on a 24 GB box would
   swap and finish *slower* than serial. `licensed_gates::jobs_for` encodes this
   and `gate_concurrency_is_bounded_by_memory_not_cores` pins it — if you
   "optimise" the bound to the core count, that test fails, by design.
3. **Parallelise at the granularity that actually holds the long pole.** Fanning
   out per *target* left `cf-fsu-model`'s lib target — ~13 gates in one binary —
   untouched. The unit that matters is the individual gate.
4. **Keep output attributable.** The reason `--test-threads=1` was there at all
   is that these gates print FOM tables through `--nocapture`, and concurrent
   tests interleaved onto one stdout destroy them. One test per process, capture
   each, print each as a labelled block. Parallelism must not cost readability —
   solve both.
5. **Warm the build once, serially, before fanning out.** Otherwise every worker
   blocks on the same cargo build lock and the fan-out buys nothing.

```bash
cargo xtask licensed-gates --run            # concurrent; bound picked from RAM
cargo xtask licensed-gates --run --jobs 1   # serial, for debugging one gate
```

**Before adding `--test-threads=1`, a `for` loop over test invocations, or any
other serialisation to a long-running tool: don't, unless you can name the
shared resource in a comment.**

## The Quality Gate

### What CI Checks

Every push and PR runs:

```yaml
cargo fmt --all -- --check                        # formatting is law
cargo xtask grade-all --skip-coverage --shard i/3 # criteria 2-7, three shards
cargo test <affected crates>                      # debug, PR-scoped
cargo nextest run --release <heavy crates>        # stochastic-physics validators
```

`grade-all` is what enforces Clippy, Documentation, Safety, Dependencies, Layer
Integrity and WASM — CI does not invoke `cargo clippy -D warnings` or
`cargo doc -D warnings` directly, it grades them. Two things that block a merge
are therefore easy to mispredict from the command line alone, so check the
grader's own output with `cargo xtask grade <crate>`.

⚠ **Not run in CI, despite what you might expect:**
- **Coverage** (criterion 1) — `--skip-coverage` on every shard. Local only.
- **`--all-features` test sweep** — deliberately excluded; Layer Integrity in
  the grader covers all-features cleanliness without re-paying the compile.
- **`no bevy in Layer 0`** as a standalone check — it is criterion 6 (Layer
  Integrity) inside the grader, not a separate job.

### What CI Cannot Check

Some gates read licensed anatomy meshes (CC BY-SA, never committed), so they are
`#[ignore]`d and **no CI run will ever execute them**. Enumerate and run them
locally — the list is derived from the source, not hand-kept:

```bash
cargo xtask licensed-gates          # what exists, grouped by crate
cargo xtask licensed-gates --run    # run it (see design/cf-fsu-geometry/BODYPARTS3D.md)
```

Run this before merging **production-code changes these gates can observe**. Two
questions, in this order:

**1. Is the change test-only or docs-only?** Then you are done — the gates cannot
observe it, in any crate.

**2. Otherwise, are the gates downstream of it?** Decide mechanically, not from
memory:

```bash
cargo xtask affected --base origin/main   # crates your diff can break
cargo xtask licensed-gates                # crates that hold licensed gates
```

If those sets intersect, run the gates. ⚠ `affected` is **path-based**, so it
reports a crate when *any* file under it changes, including a README — which is
exactly why question 1 comes first and settles it.

**Do not work from a remembered list of "the FSU crates."** The 45 gates
transitively exercise ~17 workspace crates — routing, co-design, truss and the
whole mesh pipeline, not just the FSU cone. Any prose list here would be narrower
than the truth within a release, which is the failure this tooling exists to
prevent.

Scope the run when you can: `--only <crate>` covers one crate's slice in seconds
to minutes, against hours for the full surface (2 h 21 m measured 2026-08-07). A
rule applied where it cannot matter is a rule people learn to skip, and this one
has to hold on the day it does.

### What Blocks Merging

- Any CI failure
- Any crate dropping below A-grade
- New crates without COMPLETION.md
- API changes without review

---

## Project Structure

```
CortenForge/
├── CONTRIBUTING.md      ← You are here
├── docs/                ← Project documentation
│   ├── STANDARDS.md     ← Full quality criteria
│   └── archive/         ← Historical docs
├── xtask/               ← Quality enforcement tool
│
├── cortenforge/         ← The SDK facade — one stable public crate (start here)
├── design/              ← Foundation & design kernel (8 crates)
├── mesh/                ← Mesh pipeline (10 crates incl. umbrella)
├── sim/                 ← Physics simulation (~20 crates across L0/L1)
├── tools/               ← CLIs, GUIs & app engines (Studio, cast, MSK, co-design)
└── examples/            ← Working showcases (100+ examples)
```

### Per-Crate Structure

Every crate has:

```
mesh-types/
├── Cargo.toml
├── README.md            ← Links to docs/STANDARDS.md
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
- `cf-spatial`
- `sim-*` crates (Layer 0)

**Enforcement:** `cargo tree -p <crate> | grep bevy` must return nothing.

### Layer 1: Bevy SDK

`sim-bevy` wraps Layer 0 with Bevy plugins. This is the only place Bevy appears.

---

## For AI Agents

If you are an AI agent contributing to this project:

1. **Always run `cargo xtask check` before proposing changes**
2. **Grade your work with `cargo xtask grade <crate>`**
3. **Do not mark anything complete without human review**
4. **Reference docs/STANDARDS.md for all quality decisions**
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

See CONTRIBUTING.md and docs/STANDARDS.md for full requirements.
```

---

## Code Review Process

### For Contributors

1. Run `cargo xtask grade <crate>` before requesting review
2. Ensure all automated criteria are A
3. Self-review against the API checklist in docs/STANDARDS.md
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
test(mesh-repair): add edge case tests for non-manifold edges
docs(mesh-types): add examples to all public functions
```

---

## Getting Help

- **Questions:** Open a Discussion
- **Bugs:** Open an Issue with reproduction steps
- **Features:** Open an Issue for discussion first
- **Security:** [Report a vulnerability privately](https://github.com/via-balaena/CortenForge/security/advisories/new) via GitHub Security Advisories (do not open a public issue) — see [docs/SECURITY.md](docs/SECURITY.md)

---

## License

By contributing, you agree that your contributions will be licensed under Apache-2.0.

---

## The Promise

When you contribute to CortenForge, you're not just adding code. You're maintaining a standard. The A-grade standard is what makes this project trustworthy for building real, physical things.

Every contributor before you maintained this standard. Every contributor after you will too.

**A-grade or it doesn't ship.**
