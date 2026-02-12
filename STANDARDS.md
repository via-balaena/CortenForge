# CortenForge Quality Standards

> **The A-Grade Standard:** The minimum bar for all code in this project.

---

## Philosophy

```
"An open-source SDK written in pure Rust for designing, engineering,
and manufacturing bio-inspired mechanisms and robotics."
```

We build infrastructure for bio-inspired robotics and manufacturing. A bug in our mesh code could produce an unprintable part. A flaw in our routing could short-circuit a wire harness. A mistake in our physics could injure a human interacting with a robot.

**Our code must be as reliable as the things built with it.**

This document defines what "reliable" means, quantitatively.

---

## The Seven Criteria

Every crate is graded on seven criteria. All must be A-grade before the crate is considered complete.

| # | Criterion | A Standard | Measurement |
|---|-----------|------------|-------------|
| 1 | Test Coverage | ≥75% (target: 90%) | `cargo tarpaulin` (Linux only) |
| 2 | Documentation | Zero warnings | `RUSTDOCFLAGS="-D warnings" cargo doc` |
| 3 | Clippy | Zero warnings | `cargo clippy -- -D warnings` |
| 4 | Safety | Zero unwrap/expect | grep + review |
| 5 | Dependencies | Minimal, justified | `cargo tree` + review |
| 6 | Bevy-free (Layer 0) | No bevy in tree | `cargo tree \| grep bevy` |
| 7 | API Design | Idiomatic, intuitive | Manual review |

---

## Criterion 1: Test Coverage

### A Standard: ≥75% Line Coverage (Target: 90%)

**Measurement (Linux only):**
```bash
cargo tarpaulin -p <crate> --out Html

# To match CI threshold enforcement:
cargo tarpaulin -p <crate> --fail-under 75
```

Note: `cargo tarpaulin` only works reliably on Linux. Mac/Windows users should rely on CI for coverage reporting.

**Requirements:**

- [ ] Line coverage ≥75% (target: 90%)
- [ ] All public functions have at least one test
- [ ] All error paths are tested
- [ ] Edge cases are explicitly tested:
  - Empty input
  - Single element
  - Maximum/minimum values
  - Invalid input (for functions that validate)
- [ ] Integration tests exist for cross-function workflows

**What Counts:**
- Unit tests in `#[cfg(test)]` modules
- Doc tests in `///` comments
- Integration tests in `tests/` directory

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
cargo tree -p <crate> --edges normal
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

## Criterion 6: Bevy-Free (Layer 0)

### A Standard: Zero Bevy Dependencies

**Measurement:**
```bash
cargo tree -p <crate> | grep -E "(bevy|wgpu|winit)"
# Must return empty
```

**Requirements:**

- [ ] `cargo tree` shows no bevy, wgpu, or winit
- [ ] No bevy types in public API
- [ ] No bevy traits implemented directly (only in cortenforge crate)
- [ ] Can be compiled for `--target wasm32-unknown-unknown` without bevy

**Why This Matters:**

Layer 0 crates must work in:
- CLI tools
- Web applications (WASM)
- Servers
- Embedded systems (future)
- Other game engines (future)
- Python bindings (future)

Bevy is an implementation detail of the cortenforge SDK, not the foundation.

**Layer 0 Crates:**
- `mesh-*`
- `curve-types`
- `cf-spatial`
- `route-*`
- `sensor-*`
- `ml-*`
- `sim-*` crates (Layer 0)

**Layer 1 (Bevy allowed):**
- `cortenforge`
- `sim-bevy`

---

## Criterion 7: API Design

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
║ Criterion        │ Result           │ Grade │ Threshold      ║
╠══════════════════════════════════════════════════════════════╣
║ 1. Test Coverage │ 94.2%            │   A   │ ≥75%           ║
║ 2. Documentation │ 0 warnings       │   A   │ 0              ║
║ 3. Clippy        │ 0 warnings       │   A   │ 0              ║
║ 4. Safety        │ 0 unwrap/expect  │   A   │ 0              ║
║ 5. Dependencies  │ 1 dep (nalgebra) │   A   │ minimal        ║
║ 6. Bevy-free     │ ✓ confirmed      │   A   │ no bevy        ║
║ 7. API Design    │ (manual review)  │   ?   │ checklist      ║
╠══════════════════════════════════════════════════════════════╣
║ AUTOMATED        │                  │   A   │                ║
║ OVERALL          │                  │   ?   │ needs review   ║
╚══════════════════════════════════════════════════════════════╝
```

---

## Enforcement

### Local: `cargo xtask`

```bash
cargo xtask check      # Run all checks
cargo xtask grade X    # Grade crate X
cargo xtask complete X # Record A-grade completion
```

### CI: Quality Gate

Every push triggers:
- All automated criteria checked
- Coverage computed
- Dependency tree analyzed
- PR blocked if any criterion fails

### Review: Human Required

Criterion 7 (API Design) requires human review. The `cargo xtask complete` command prompts for this.

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
