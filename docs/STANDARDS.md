# CortenForge Quality Standards

> **The A-Grade Standard:** The minimum bar for all code in this project.

---

## Philosophy

```
"An open-source SDK written in pure Rust for the mechatronics
and simulation space — design, simulation, and manufacturing."
```

We build an SDK for mechatronics and simulation. The domains we serve — design, simulation, fabrication, control — demand correctness. Our code must be as reliable as the things built with it.

This document defines what "reliable" means, quantitatively.

---

## The Eight Criteria

Every crate is graded on eight criteria. All must be A-grade before the crate is considered complete.

| # | Criterion | A Standard | Measurement |
|---|-----------|------------|-------------|
| 1 | Test Coverage | ≥75% (A); ≥90% (A+) | LLVM source-based coverage, scoped to the crate (cross-platform) |
| 2 | Documentation | Zero warnings | `RUSTDOCFLAGS="-D warnings" cargo doc` |
| 3 | Clippy | Zero warnings | `cargo clippy -- -D warnings` |
| 4 | Safety | Zero safety violations | grep (6 patterns) + review |
| 5 | Dependencies | Minimal, justified | `cargo tree` + review |
| 6 | Layer Integrity | Tier rules respected | `cargo tree` × 6 configs + tier rules |
| 7 | WASM Compatibility | L0 builds for `wasm32-unknown-unknown` | `cargo check --target wasm32-unknown-unknown --no-default-features` |
| 8 | API Design | Idiomatic, intuitive | Manual review |

Criteria 1–7 are automated by `cargo xtask grade <crate>`. Criterion 8 (API Design) requires human review. Criteria 6 and 7 are tier-aware — every crate declares its tier in `Cargo.toml` (`[package.metadata.cortenforge] tier = "..."`); the grader enforces tier rules at PR time. See Criterion 6 for tier definitions.

---

## Criterion 1: Test Coverage

### A Standard: ≥75% Line Coverage; A+ Standard: ≥90%

**Measurement:**
```bash
cargo xtask grade <crate>          # criterion 1 reports the number
```

Requires the `llvm-tools` rustup component (`rustup component add
llvm-tools-preview`); no `cargo-llvm-cov` install is needed. Works on Linux,
macOS (including Apple Silicon), and Windows.

**Instrumentation is scoped to the crate being measured** (and its own test
targets, so their binaries emit usable profiles). `RUSTFLAGS` applies
to every unit cargo builds, so instrumenting through `cargo llvm-cov`
instrumented the whole dependency tree — and the report was then filtered back
down to the crate's own files, discarding all of it. Because those lines were
never counted, scoping the build cannot move the number; measured both ways,
`sim-types` returns the same 192/271 lines and `cf-fsu-model` the same 84.2%.
What it moves is the clock: `cf-fsu-model`'s grade went 3105 s → 17 s.

⚠ The tax that remains is the crate's *own* code, and it is large. Measured
per lib suite, clean vs instrumented: `mesh-printability` 0.25 s → 41 s
(**164×**), `mesh-repair` 0.32 s → 392 s (**1226×**). It scales with how much of
the suite is instrumented inner-loop code, so treat it as that range rather than
one figure. A crate that is a thin layer over heavy dependencies
(`cf-fsu-model`) gets nearly all of it back; a crate whose own code is the hot
path (`mesh-repair`) does not. That residue is intrinsic to source-based
coverage, not something the grader is doing wrong.

**Unit AND integration tests count.** The measurement builds `--lib --tests`,
so a crate that keeps its tests in `tests/` is credited for them. It used to
build `--lib` alone, which reported where a crate's tests *live* rather than
whether its production code is exercised — `cf-geometry`, whose 186 tests are
in `tests/`, read 74.1 % (B) while its `mesh.rs` was at 7.27 % measured that way
and 100 % measured properly.

⚠ Two limits worth knowing. **Doctests are not counted** — they are built by
rustdoc, not rustc, so the per-crate instrumentation wrapper never sees them; a
crate leaning on doc examples reads low here (`mesh-repair` has 27 doctests,
`cf-design` 19). And **only the `--lib` binary gates pass/fail** inside this
criterion: an instrumented wall-clock or memory-ceiling assertion fails because
it is being measured, so integration suites contribute coverage without gating.
They are still gated — `grade`'s second pass runs the whole suite
uninstrumented, doctests included.

**When Coverage reports N/A rather than a percentage.** Four cases. Each says
there is nothing to measure; none of them waives the threshold on code that
exists. A crate with **no lib target** has nothing to instrument ("(bin-only)");
a crate opting into `grading_profile = "integration-only"`
declares it has no testable lib API; a crate whose files map no lines reports
"(no production lines)"; and a crate whose `src/` declares **no items at all
outside `#[cfg(test)]`** reports the same. The last is the `*-benches` shape — a
lib that is only a doc comment, with every benchmark in `benches/*.rs` under
`harness = false`. With no functions there is no coverage map, so no `.profraw`
is written, which is the *identical symptom* to instrumentation failing to reach
a crate that does have code. Measured on `sim-core-benches`: under blanket
`-C instrument-coverage` its test binary carries zero `__llvm_covmap` sections
and emits no profile, so this is intrinsic rather than a scoping artifact. The
two causes are separated by reading the source — the empty-crate verdict needs a
clean parse that finds nothing, so an unreadable file keeps the failure report.
**Benchmarks are never counted as coverage**; `cargo test --lib --tests` does not
build them, by design.

⚠ **"(bin-only)" is now a measured fact, not a guess from the crate's
directory.** It used to be asserted for every `Example`/`Xtask`/`tools` crate
without checking, and it was **false for 13 of the 18 crates under `tools/`** —
**9718 production lines** that no coverage run ever touched, one of them
`cf-codesign` (the co-design optimizer). All of them reported `—` and passed,
because N/A is skipped by the automated roll-up. Measuring them showed **5505
of those lines, 56.6 %, were covered all along** — the tests existed and
nothing was reading them, so the skip hid a measurement gap far more than a
quality one. (9718 is the coverage instrument's own count. A first pass
reported 13 265 from a source-line heuristic, overstating by 36 % because it
counted `use` lines, attributes and braces that llvm-cov never maps; the size
of an unmeasured gap belongs in the units of the missing measurement.)

The profile still decides the Clippy and Safety relaxations by path;
it no longer decides whether a library gets measured. A crate is skipped here
only when Cargo would build no lib target for it — no `[lib]` table, no
`src/lib.rs`, or `autolib = false`. Same failure shape as the fail-open dead
zone closed in §7: a gate whose green meant "not measured" while its text named
a property nobody had checked.

**Report-only crates.** Turning that skip off lit 14 crates at once, so the
threshold is deferred for the six that fail while it is enforced for everyone
else. A deferred crate prints its real percentage with `(report-only)` beside
it and a detail line naming the grade it *would* have taken; only the
**threshold** is waived. Failing tests, Clippy, Safety, Documentation and
Dependencies gate these crates exactly as they gate every other, and a red test
run is never waivable. So green here means "measured, enforcement deferred" —
never "not measured", which is the distinction §7 exists to protect.

The list is a finite to-do in `grade.rs`, not a rule: a crate added to `tools/`
tomorrow is enforced from its first grade, and enforcing one of these is a
deletion. A crate that was **already** being measured and failing is never
added — that would switch off a gate that fires today rather than defer one
that just appeared, which is why `cf-viewer` is not on it.

The 2026-08-16 census, per crate, is the evidence the list rests on. **Eight of
the fourteen newly-measured crates already passed** — cf-mjcf-emit 97.8 %,
cf-msk-lib 95.7 %, cf-msk-fit 94.7 %, cf-osim 94.6 %, cf-studio-core 90.3 %,
cf-codesign 87.2 %, cf-cast-cli 79.9 %, cf-studio 75.2 % — so what the skip hid
was mostly a measurement gap rather than a quality one. The six deferred are
cf-scan-prep-core (0.0 % of 1560 lines, no test in the crate),
cf-studio-gui (14.3 %, but 93.9 % over its library — 1657 lines are a Bevy GUI
binary), example-ml-shared (0.0 % of 131), pbit-analyze (69.6 %, 25 lines
short), cf-anthro (71.1 %, 13 short) and cf-studio-engine (74.969 %, **one**
line short).

⚠ **`xtask grade` cannot be run concurrently with itself.** The instrumented
build uses one shared coverage target directory and resets it whenever the
crate under measurement changes, so two grades running at once delete each
other's build and surface as "measurement failed" / "Directory not empty".
Eight of the fifteen crates in the first census pass failed this way and had to
be re-measured serially. CI is unaffected — `grade-all --shard i/N` fans out
across separate jobs — but a local sweep must be serial.

**Percentages are truncated, not rounded**, so the printed figure is never
above the graded one. `cf-studio-engine` at 605/807 = 74.969 % would otherwise
print "75.0%" in the same row as its `B`.

**Library lines and binary lines are reported apart.** The graded percentage
spans both, unchanged. But a crate with a large `main.rs` is held to a bar its
binary structurally cannot clear — `.run()`, `Cli::parse()` and an event loop
are not unit-testable — so when a binary target contributes lines, the detail
line also reports the figure over library lines alone, and the triage table
marks the binary root `(binary target)`. Measured on `cf-viewer`: 33.8 % overall
against 57.7 % over its library, with 460 of 1082 production lines in
`src/main.rs` at 1.5 %. The census found a second, stronger instance:
`cf-studio-gui` reads 14.3 % overall and **93.9 % over its library**, because
1657 of its 1955 lines are a Bevy GUI binary. Reported only; **excluding binary
lines from the grade is deliberately not done here**, because it would make
`main.rs` a place where logic stops being measured — the dead zone this section
just finished closing. The number exists so that decision can be taken on
evidence.

**Where the uncovered lines are.** A percentage says a crate needs tests; it
does not say where to write them. `grade` therefore prints a per-file breakdown
under the table whenever coverage ran and something is uncovered — worst first,
so the biggest win is the top row:

```text
  Coverage triage — 2 of 3 measured file(s) hold 79 uncovered production line(s):
      uncovered  covered  file
             48    72.7%  src/config.rs
             31    64.4%  src/body.rs
```

That is `sim-types`' real output at 70.8 % (192/271), **as measured
2026-08-15**; #776 has since taken the crate to 97.7 % by testing those two
files, so the shape is the illustration and the numbers are a snapshot. The
third measured file, `src/error.rs`, is at 100 % and so is not a row — it
appears under `--json`, which lists every measured file.

This is the *same* measurement the percentage comes from, split by file rather
than summed — the per-file numerators and denominators add up to the crate's, by
construction. It costs no extra run, which is the point: a coverage run is
minutes per crate, so a breakdown behind a second invocation (or behind a flag
that is easy to forget) would mostly not be taken.

Two scoping rules follow from "a row is somewhere to go and write a test".
Fully-covered files are omitted, and so are files with no production lines at
all — a file that is entirely `#[cfg(test)]`, or `tests/` integration source,
which is excluded from the ratio and would otherwise send a reader off to write
tests for a test. Those lines are still reported in the excluded count.

A row for a file the grader could not read or parse is marked `⚠ unparsed, so
test lines are counted here` (`test_lines_counted` under `--json`). With no
spans to subtract, such a file keeps its `#[cfg(test)]` lines, and `#[ignore]`d
gates among them read as uncovered production code — so the row overstates the
gap by an amount nothing here can bound, and can outrank every honest row. The
crate-level detail already reports *how many* files this hit; the marker says
which, because the ranking is what a reader acts on.

The printed table stops at 20 rows and states what it cut — on `sim-mjcf`,
`… 19 more file(s) hold 364 uncovered line(s); --json lists every file` —
because a bound nobody is told about reads as the whole list. That crate is
also the evidence the bound is cheap: its printed 20 rows hold 91 % of its
4007 uncovered lines. `cargo xtask grade <crate> --json` carries every
measured file unbounded, under a `coverage_files` key that is **absent rather
than empty** when coverage did not run — verified on both routes that reach
that state, `--skip-coverage` and a profile whose coverage is N/A — so a
consumer can tell "measured, all covered" from "never measured".

**Requirements:**

- [ ] Production line coverage ≥75% (A grade — ships)
- [ ] Production line coverage ≥90% (A+ grade — gold standard)
- [ ] All public functions have at least one test
- [ ] All error paths are tested
- [ ] Edge cases are explicitly tested:
  - Empty input
  - Single element
  - Maximum/minimum values
  - Invalid input (for functions that validate)
- [ ] Integration tests exist for cross-function workflows

**What Counts** (as a *test*, i.e. something that can cover a line):
- Unit tests in `#[cfg(test)]` modules
- Doc tests in `///` comments
- Integration tests in `tests/` directory

**What Is Measured** (the lines in the ratio): **production lines only.**

The instrumented run measures *test* binaries, so a crate's own
`#[cfg(test)]` code appears in the report next to the code it exercises.
`cargo xtask grade` subtracts it from both sides of the ratio, because counting
it measures two unrelated things at once:

- test bodies that run are ~100% covered, so they **inflate** the number —
  measured across seven crates, worth up to +12 points;
- test bodies that *cannot* run — `#[ignore]`d licence-gated gates and the
  helpers reachable only from them — sit in the denominator contributing zero,
  so improving a crate's instrumentation **lowers** its letter.

⚠ Consequence: the grade's percentage is deliberately **not** the number a bare
`cargo llvm-cov -p <crate> --fail-under-lines 75` prints — that one still counts
test code, and its denominator spans the whole instrumented tree. When the two
disagree, the grade is the one that answers "how much of what I ship is tested".

**Exceptions:** None for Layer 0 crates.

### Example Test Structure

```rust
#[cfg(test)]
mod tests {
    use super::*;

    // Happy path
    #[test]
    fn bounds_of_simple_mesh() {
        let mesh = create_unit_cube();
        let bounds = mesh.bounds();
        assert_eq!(bounds.min, Point3::new(0.0, 0.0, 0.0));
        assert_eq!(bounds.max, Point3::new(1.0, 1.0, 1.0));
    }

    // Edge case: empty
    #[test]
    fn bounds_of_empty_mesh() {
        let mesh = IndexedMesh::empty();
        assert!(mesh.bounds().is_empty());
    }

    // Edge case: single vertex
    #[test]
    fn bounds_of_single_vertex() {
        let mesh = IndexedMesh::from_vertices(vec![
            Vertex::new(Point3::new(1.0, 2.0, 3.0))
        ]);
        let bounds = mesh.bounds();
        assert_eq!(bounds.min, bounds.max);
    }

    // Error path
    #[test]
    fn repair_rejects_invalid_indices() {
        let mesh = mesh_with_invalid_indices();
        let result = repair(&mesh, &Config::default());
        assert!(matches!(result, Err(RepairError::InvalidIndices(_))));
    }
}
```

---

## Criterion 2: Documentation

### A Standard: Zero Warnings, Complete Coverage

**Measurement:**
```bash
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps -p <crate>
```

**Requirements:**

- [ ] Every public item has a doc comment (`///` or `//!`)
- [ ] Every public function has at least one `# Examples` section
- [ ] Module-level docs (`//!`) explain purpose and typical usage
- [ ] All intra-doc links resolve (no broken `[links]`)
- [ ] No `missing_docs` warnings
- [ ] No `broken_intra_doc_links` warnings

**Doc Comment Structure:**

```rust
/// Short one-line description.
///
/// Longer explanation of what this does, when to use it,
/// and any important details.
///
/// # Arguments
///
/// * `mesh` - The mesh to process
/// * `config` - Configuration options
///
/// # Returns
///
/// The processed mesh, or an error if processing failed.
///
/// # Errors
///
/// Returns [`RepairError::InvalidMesh`] if the mesh has no vertices.
///
/// # Examples
///
/// ```
/// use mesh_repair::{repair, RepairConfig};
/// use mesh_types::IndexedMesh;
///
/// let mesh = IndexedMesh::from_vertices(vertices);
/// let repaired = repair(&mesh, &RepairConfig::default())?;
/// assert!(repaired.is_watertight());
/// # Ok::<(), mesh_repair::RepairError>(())
/// ```
///
/// # Panics
///
/// This function does not panic. (Or: Panics if X, which should never happen because Y.)
pub fn repair(mesh: &IndexedMesh, config: &RepairConfig) -> Result<IndexedMesh, RepairError> {
    // ...
}
```

**Module-Level Docs:**

```rust
//! Mesh repair utilities for CortenForge.
//!
//! This crate provides functions for repairing common mesh defects:
//! - Duplicate vertices (see [`weld_vertices`])
//! - Degenerate triangles (see [`remove_degenerates`])
//! - Holes (see [`fill_holes`])
//!
//! # Quick Start
//!
//! ```
//! use mesh_repair::{repair, RepairConfig};
//!
//! let repaired = repair(&mesh, &RepairConfig::default())?;
//! ```
//!
//! # Architecture
//!
//! This is a Layer 0 crate with no Bevy dependencies. It can be used
//! standalone or through the `cortenforge` Bevy SDK.
```

---

## Criterion 3: Clippy

### A Standard: Zero Warnings

**Measurement:**
```bash
cargo clippy -p <crate> --all-targets --all-features -- -D warnings
```

**Requirements:**

- [ ] Zero clippy warnings
- [ ] No `#[allow(clippy::...)]` without justification comment
- [ ] Pedantic lints encouraged (not required): `#![warn(clippy::pedantic)]`

**Justified Allows:**

```rust
// Justification: This index is guaranteed valid by the loop bounds above.
// The bounds check would add overhead in this hot path.
#[allow(clippy::indexing_slicing)]
let vertex = &vertices[i];
```

**Common Issues and Fixes:**

| Warning | Fix |
|---------|-----|
| `clippy::unwrap_used` | Use `?` or `ok_or()` |
| `clippy::expect_used` | Use `?` or `ok_or()` |
| `clippy::clone_on_copy` | Remove `.clone()` |
| `clippy::needless_return` | Remove `return` keyword |
| `clippy::redundant_closure` | Use function directly |

---

## Criterion 4: Safety

### A Standard: Zero Runtime Panics in Library Code

**Measurement:**
```bash
# In xtask, we grep for these patterns
grep -r "\.unwrap()" src/
grep -r "\.expect(" src/
grep -r "panic!" src/
grep -r "unreachable!" src/
grep -r "todo!" src/
grep -r "unimplemented!" src/
```

**Requirements:**

- [ ] Zero `unwrap()` in library code
- [ ] Zero `expect()` in library code
- [ ] Zero `panic!()` except for genuinely impossible states
- [ ] Zero `todo!()` or `unimplemented!()` in shipped code
- [ ] All `unreachable!()` have comments explaining why

**Allowed:**
- `unwrap()` in tests
- `unwrap()` in examples
- `expect()` in `build.rs`
- `panic!()` with justification comment for impossible states

**Patterns to Use Instead:**

```rust
// Instead of: value.unwrap()
// Use:
let value = optional.ok_or(MyError::NotFound)?;

// Instead of: vec[index].unwrap()
// Use:
let value = vec.get(index).ok_or(MyError::IndexOutOfBounds(index))?;

// Instead of: string.parse::<i32>().unwrap()
// Use:
let value: i32 = string.parse().map_err(|_| MyError::ParseFailed)?;

// For truly impossible states (with justification):
// This is unreachable because we just checked vec.len() > 0 above.
#[allow(clippy::unreachable)]
let first = vec.first().unwrap_or_else(|| unreachable!("vec is non-empty"));
```

**Unsafe Code:**

- [ ] Zero `unsafe` blocks without justification
- [ ] Each `unsafe` block has a `// SAFETY:` comment
- [ ] Each `unsafe` block is reviewed by maintainer
- [ ] Prefer safe alternatives even if slightly slower

```rust
// SAFETY: We've verified that:
// 1. The pointer is non-null (checked above)
// 2. The data is properly aligned (guaranteed by allocator)
// 3. The data is valid for the lifetime 'a (bounded by input lifetime)
unsafe {
    &*ptr
}
```

---

## Criterion 5: Dependencies

### A Standard: Minimal and Justified

**Measurement:**
```bash
# Dep count (informational):
cargo metadata --format-version 1 --no-deps
# Justification check (hard gate):
# Every dependency in Cargo.toml must have a justification comment
```

**Requirements:**

- [ ] Each dependency has documented justification in Cargo.toml
- [ ] No duplicate functionality (e.g., both `rand` and `fastrand`)
- [ ] Heavy dependencies are feature-gated
- [ ] Version constraints are reasonable (not overly restrictive)

**Documenting Dependencies:**

```toml
[dependencies]
# Core math library - matches parry3d for collision geometry compatibility
nalgebra = { workspace = true }

# Error handling - standard for library error types
thiserror = { workspace = true }

# File parsing - only STL support, lightweight
# Justification: We could write our own, but this is battle-tested
stl_io = "0.7"

[dependencies.heavy-dep]
version = "1.0"
optional = true
# Justification: Only needed for advanced feature X
# Adds ~2MB to binary, so gated behind feature

[features]
default = []
advanced = ["heavy-dep"]
```

**Dependency Review Checklist:**

- [ ] Is this dependency maintained? (Check last commit, issues)
- [ ] Is there a lighter alternative?
- [ ] Could we implement this ourselves reasonably?
- [ ] Does it pull in transitive dependencies we don't want?
- [ ] Is it compatible with our MSRV?

---

## Criterion 6: Layer Integrity

### A Standard: Tier Rules Respected (Banned-Prefix + Dep-Count Caps)

Replaces the prior "Bevy-free (Layer 0)" criterion (which only checked one prefix in one graph). Layer Integrity is tier-aware, checks six dep-graph configurations, and enforces both banned-prefix exclusions and dep-count maximums per tier.

**Tier System.** Every workspace crate (under prefixes `sim-*`, `mesh-*`, `cf-*`, `cortenforge*`) declares its tier in `Cargo.toml`:

```toml
[package.metadata.cortenforge]
tier = "L0"  # one of: "L0", "L0-io", "L0-integration", "L1"

# Optional: features that promote this crate to a stricter (heavier) tier
# when enabled. Distinguishes intentional opt-in from accidental leak.
tier_up_features = { gpu-probe = "L0-io" }
```

| Tier | Definition | Release max | Test max | Banned prefixes |
|---|---|---:|---:|---|
| **L0** | Pure compute substrate. Math, types, algorithms, in-memory data structures. No file I/O, no graphics, no GPU compute, no game-engine integration. WASM-buildable. | 80 | 100 | `bevy*`, `winit`, `wgpu*`, `image*`, `zip*`, `zstd*`, `sim-mjcf`, `sim-urdf`, `mesh-io`, `criterion`, `plotters*` |
| **L0-io** | Format parsers, asset loaders, GPU compute kernels. Allowed format-specific heavy chains and `wgpu` compute (NOT bevy). WASM-buildable not required. | 200 | 220 | `bevy*`, `winit` |
| **L0-integration** | Composes L0 + L0-io into higher-level abstractions. Bevy-free. Inherits L0-io weight legitimately. | 200 | 220 | `bevy*`, `winit` |
| **L1** | Visualization, ECS integration, interactive runtime. Bevy/winit/wgpu allowed unconditionally. | unbounded | unbounded | (none) |

Numbers track plan §5.2; the architectural rationale lives in `sim/docs/L0_architectural_plan.md` §2.1 and §5.2. Plan §2.1 proposes tighter caps (60/180/180) post-foundation; the grader will re-tune at the next foundation review.

**Measurement (per crate, per PR):**
```bash
cargo run -p xtask --release -- grade <crate>
```

The grader runs `cargo tree` six times per in-scope crate — three feature configs (`--no-default-features`, default, `--all-features`) × two graph kinds (release `-e normal`, dev-test `-e normal,dev`) — then checks each config against the declared tier's banned-prefix list and dep-count cap. Findings name the violating dep + which config + which graph it appeared in.

**Requirements:**

- [ ] `Cargo.toml` declares `[package.metadata.cortenforge].tier`
- [ ] Release graph (no-default + default + all-features) contains no banned prefix for the declared tier
- [ ] Dev graph (release + dev-deps, all three feature configs) contains no banned prefix for the declared tier
- [ ] Release dep count ≤ tier's `release_max` across all three feature configs
- [ ] Dev dep count ≤ tier's `test_max` across all three feature configs
- [ ] Tier-up features (e.g. sim-soft's `gpu-probe → L0-io`) are explicitly declared in `tier_up_features` if a feature legitimately pulls heavier deps

**Why This Matters:**

L0-pure crates must work in CLI tools, web applications (WASM), servers, embedded systems, other game engines, and Python bindings. The four-tier system prevents architectural drift: a leak (bevy_ecs accidentally added to an L0 crate's `--all-features` graph) and an intentional tier-up (gpu-probe pulling wgpu) become structurally distinguishable. Without the tier-up declaration, the leak fails grading; with it, the heavier graph is permitted under the higher tier's rules.

The dev-graph check specifically prevents the "+130 transitive deps in test compile" pattern that previously dominated CI test-compile times.

**Tier Assignment:** Tier metadata lives in each crate's `Cargo.toml`. To find a crate's tier, look at its `[package.metadata.cortenforge]` block. The directory layout (`sim/L0/`) is historical and does not determine tier — multiple tiers coexist under that path.

---

## Criterion 7: WASM Compatibility (L0 only)

### A Standard: L0 Crates Build Cleanly for `wasm32-unknown-unknown`

L0 is the binary-clean foundation for embedded, web, and headless consumers. The grader enforces that every L0 crate (only) compiles for `wasm32-unknown-unknown` under `--no-default-features`. L0-io / L0-integration / L1 are NotApplicable (skipped, not graded on this criterion).

**Measurement:**
```bash
cargo check -p <crate> --target wasm32-unknown-unknown --no-default-features
# Must exit 0
```

The grader runs this automatically as part of `cargo xtask grade <crate>` when the declared tier is L0. If the wasm32 target is not installed locally, the criterion reports as `(target n/a)` (Manual grade, not a failure) — install with `rustup target add wasm32-unknown-unknown`.

**Requirements:**

- [ ] Tier == L0 (otherwise the criterion is NotApplicable)
- [ ] `cargo check --target wasm32-unknown-unknown --no-default-features` exits 0
- [ ] No use of `std::` items missing on wasm (e.g., `std::process`, `std::os::unix`)
- [ ] Non-deterministic / system-time / RNG entry points work via wasm-compatible backends (the workspace registers `getrandom = { version = "0.3", features = ["wasm_js"] }` and a `.cargo/config.toml` rustflag selecting the wasm_js backend; affected crates inherit this via target-conditional dep blocks)

**Why This Matters:**

STANDARDS.md prior versions listed "compiles for wasm32-unknown-unknown" as an L0 requirement under the Bevy-free section, but enforcement was a CI warning, not a hard gate. The grader is now the single source of truth — CI no longer runs a standalone WASM job (plan §6.4).

**Common Issues:**
- `getrandom 0.3.x` requires both the rustflag (`getrandom_backend="wasm_js"`) and the cargo feature (`wasm_js`) — neither alone is sufficient.
- `image`, `zip`, `mesh-io`, `sim-mjcf` and similar I/O-heavy deps are L0-banned anyway, so wasm-incompatibility there is moot at L0.
- Float intrinsics or SIMD: nalgebra/simba's wasm story is solid under default features; no special action needed for typical L0 usage.

---

## Criterion 8: API Design

### A Standard: Idiomatic, Intuitive, Documented

**This criterion requires manual review.**

**Checklist:**

- [ ] Follows [Rust API Guidelines](https://rust-lang.github.io/api-guidelines/)
- [ ] Naming is consistent with standard library conventions
- [ ] Naming is consistent with rest of CortenForge
- [ ] Types are appropriately generic or concrete
- [ ] Errors are meaningful and actionable
- [ ] `#[must_use]` on functions with important return values
- [ ] `#[non_exhaustive]` on enums that may grow
- [ ] Builder pattern for complex construction
- [ ] No unnecessary allocations in hot paths

**Naming Conventions:**

| Type | Convention | Example |
|------|------------|---------|
| Types | PascalCase | `IndexedMesh`, `RepairConfig` |
| Functions | snake_case | `compute_bounds`, `load_stl` |
| Constants | SCREAMING_SNAKE | `MAX_VERTICES`, `DEFAULT_TOLERANCE` |
| Modules | snake_case | `mesh_types`, `vertex` |
| Traits | PascalCase, often `-able` or `-er` | `Meshable`, `Loader` |
| Type parameters | Single uppercase or descriptive | `T`, `V`, `Backend` |

**Error Design:**

```rust
/// Errors that can occur during mesh repair.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum RepairError {
    /// The mesh has no vertices.
    #[error("mesh has no vertices")]
    EmptyMesh,

    /// A vertex index is out of bounds.
    #[error("vertex index {index} is out of bounds (max: {max})")]
    IndexOutOfBounds {
        index: u32,
        max: u32,
    },

    /// The mesh has non-manifold edges.
    #[error("mesh has {count} non-manifold edges")]
    NonManifold {
        count: usize,
    },
}
```

**Builder Pattern:**

```rust
/// Configuration for mesh repair operations.
#[derive(Debug, Clone)]
pub struct RepairConfig {
    weld_threshold: f64,
    fill_holes: bool,
    max_hole_edges: usize,
}

impl RepairConfig {
    /// Create a new configuration with default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the vertex welding threshold.
    #[must_use]
    pub fn weld_threshold(mut self, threshold: f64) -> Self {
        self.weld_threshold = threshold;
        self
    }

    /// Enable or disable hole filling.
    #[must_use]
    pub fn fill_holes(mut self, enabled: bool) -> Self {
        self.fill_holes = enabled;
        self
    }

    /// Set the maximum hole size to fill.
    #[must_use]
    pub fn max_hole_edges(mut self, edges: usize) -> Self {
        self.max_hole_edges = edges;
        self
    }
}

impl Default for RepairConfig {
    fn default() -> Self {
        Self {
            weld_threshold: 1e-6,
            fill_holes: true,
            max_hole_edges: 100,
        }
    }
}
```

---

## Grading Scale

| Grade | Meaning | Action |
|-------|---------|--------|
| **A** | Meets all requirements | Ready to ship |
| **B** | Minor issues | Refactor before continuing |
| **C** | Significant gaps | Major refactor required |
| **F** | Does not meet standards | Rewrite or reject |

**Only A-grade crates ship. No exceptions.**

---

## The Grade Command

```bash
$ cargo xtask grade mesh-types

╔══════════════════════════════════════════════════════════════╗
║                    GRADING: mesh-types                        ║
╠══════════════════════════════════════════════════════════════╣
║ Criterion          │ Result           │ Grade │ Threshold    ║
╠══════════════════════════════════════════════════════════════╣
║ 1. Coverage        │ 94.2%            │  A+   │ ≥75%(A),≥90%(A+) ║
║ 2. Documentation   │ 0 warnings       │   A   │ 0 warnings   ║
║ 3. Clippy          │ 0 warnings       │   A   │ 0 warnings   ║
║ 4. Safety          │ 0 violations     │   A   │ 0 violations ║
║ 5. Dependencies    │ 1 dep, all just. │   A   │ all justified║
║ 6. Layer Integrity │ ✓ confirmed      │   A   │ tier rules   ║
║ 7. WASM Compat     │ ✓ builds         │   A   │ wasm32 OK    ║
║ 8. API Design      │ (manual review)  │   ?   │ checklist    ║
╠══════════════════════════════════════════════════════════════╣
║ AUTOMATED          │                  │   A   │              ║
║ OVERALL            │                  │   ?   │ needs review ║
╚══════════════════════════════════════════════════════════════╝
```

`cargo xtask grade-all` runs the full sweep over all 301 workspace crates and reports a workspace-level pass/fail. It's the same gate CI runs (with `--skip-coverage` for runtime; coverage is a local-only gate per the note below).

The sweep grades every crate even when some cannot be graded: a crate whose grade errors out is recorded and the run continues, so one broken crate never hides the verdict on the rest of its shard. Those crates are reported in their own section rather than counted as failures, since "could not measure" is a different problem from "measured, and it is bad" — but either one fails the sweep. A crate that was never measured is never absorbed into a green result.

The criteria that grade by *scanning source files* — 3 (unjustified `#[allow]`) and 4 (Safety) — fail closed. Both score a crate by counting violations across its `src/` tree, so any file the scan fails to reach or read is a file whose violations are never counted, and the crate would be graded A for a reason nobody measured. An unwalkable directory or an unreadable source file therefore aborts the grade with an error rather than producing a quietly clean result, on the same principle as the unreadable-file rule under Criterion 1 above. (Criterion 3 skips its `#[allow]` scan altogether for the `Example` and `Xtask` profiles, per the relaxation in §3; the guarantee covers that scan wherever it runs, not the profiles that opt out of it.)

Criterion 3's *clippy* half fails closed for the same reason, by a different mechanism. A zero-warning count means nothing on its own: clippy finding no problems and clippy never running both produce zero, and the exit code cannot separate them because clippy exits non-zero merely for finding warnings. An unresolvable `-p` exits 101 having written nothing at all to stdout — byte-for-byte what a flawless crate produces. So the grader requires cargo's `build-finished` record before it will report a count, and refuses in two cases: when no such record arrives (the invocation never completed), and when it reports `success: false` with no diagnostic inside the graded crate. That second case means the build failed but nothing in the stream attributes the failure to this crate — it may be a dependency, or a failure that carries no span at all, since cargo reports "could not compile" on stderr rather than as a structured diagnostic. Either way the crate was not analysed, so zero proves nothing. It is deliberately not graded F: an unattributable failure is not evidence against this crate. A `success: false` *with* diagnostics inside the crate is that crate's own compile errors, and is graded F.

---

## Enforcement

### Local: `cargo xtask`

```bash
cargo xtask check      # Run all checks
cargo xtask grade X    # Grade crate X
cargo xtask complete X # Record A-grade completion
```

### CI: Quality Gate

Every push to `main`/`develop` and every PR triggers parallel CI jobs (`.github/workflows/quality-gate.yml`). The grader (`cargo xtask grade-all`) is the policy source of truth — CI invokes it directly rather than running parallel bash/awk reimplementations of the same checks.

| Job | Purpose |
|---|---|
| Format | `cargo fmt --check` strict |
| xtask Grade | `cargo xtask grade-all --skip-coverage --quiet` — runs Criteria 2–7 over every workspace crate. Coverage is excluded for runtime (see below). |
| Tests (debug — shard N/3) | `cargo test` (default features, debug profile) on the ~23 light-weight workspace crates, sharded 3 ways (balanced by dependency tree — the Bevy and Slint GUI trees isolated one-per-shard) since a foundational `sim-core` change makes this the compile-bound long pole. |
| Tests (release, heavy) | `cargo test --release` on the 5 crates whose default test suite contains heavy stochastic-physics validators (sim-ml-chassis, sim-thermostat, sim-rl, sim-conformance-tests, sim-opt). Per `feedback_release_mode_heavy_tests.md` and plan §6.5: these tests do 90M–600M physics steps each and run 5–10× slower in debug. |
| Cross-OS Tests (macOS, Windows) | Default-features test on the 3 platform-sensitive crates (sim-core for FP determinism, sim-mjcf for XML/file paths, mesh-io for multi-format file paths). Other crates' cross-OS coverage is provided by ubuntu in the tests-debug + tests-release jobs. |
| Feature Combos | Non-default feature paths. Currently sim-soft `gpu-probe` (the only `tier_up_features` declarer); add new combos here as features land. |
| Dependencies | `cargo-deny` license + source audit. |
| Semver | `cargo-semver-checks` against the prior published release. |
| SBOM | Supply-chain manifest generation. |
| Quality Gate | Aggregator job; `needs:` all of the above and gates merge. |

**PR is blocked if any job fails.**

**What CI does NOT run** (intentional, per plan §6):
- `--all-features` test sweep — Layer Integrity in the grader enforces all-features cleanliness; re-running tests under all-features would just re-pay the bevy_ecs / image / zip / criterion compile cost on every consumer's test build for no additional signal.
- Standalone WASM job — the WASM Compatibility criterion (#7) is the single source of truth.
- Coverage gate — still minutes per crate for crates whose own code is the hot path (`mesh-repair`, ~6 min), so running it on 232 crates exceeds the wall-time budget even after instrumentation was scoped to the measured crate. Coverage is enforced locally via `cargo xtask grade <crate>`. ⚠ The nightly `coverage` job in `scheduled.yml` does **not** enforce this criterion: it runs `cargo tarpaulin --workspace --all-features --fail-under 75`, which counts test code and pools the whole workspace into one ratio. That job never got the migration this section describes, so no CI job measures the number defined here. Resolving the split is its own change.

### CI Wall-Time Budget

Honest budget on free GitHub-hosted runners is **20–25 minutes**. Reference run (PR #216, post-foundation): **22m37s wall-time**, with the long-pole jobs being `xtask Grade` (≈21m, dominated by wasm32 cold-compile across 18 L0 crates) and `Tests (release, heavy)` (≈22m, cold-cache --release compile + 5 heavy-validator tests). Both run in parallel; wall-time = max(jobs).

Plan §6.5 documents the bucketing rationale + when this budget gets revisited (e.g., if PR cache-restore lands, if xtask Grade gets split into fast-grade + wasm-grade parallel jobs, or if we move to paid runners).

### Review: Human Required

Criterion 8 (API Design) requires human review. The `cargo xtask complete` command prompts for this.

---

## Versioning

All crates follow semantic versioning:
- **MAJOR:** Breaking API changes
- **MINOR:** New features, backward compatible
- **PATCH:** Bug fixes only

During 0.x development, breaking changes are allowed in minor versions.

---

## References

- [Rust API Guidelines](https://rust-lang.github.io/api-guidelines/)
- [Effective Rust](https://www.lurklurk.org/effective-rust/)
- [The Power of Ten Rules](https://en.wikipedia.org/wiki/The_Power_of_10:_Rules_for_Developing_Safety-Critical_Code)
- [NASA JPL Coding Standard](https://standards.nasa.gov/)
