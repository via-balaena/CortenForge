//! Discover and run the example-validators red-or-green.
//!
//! # The validation contract
//!
//! An `examples/**` crate plays one of two roles, declared in its `Cargo.toml`
//! under `[package.metadata.cortenforge] example_kind` (validators live under
//! `examples/fundamentals/` today):
//!
//! - `validator` — the binary self-checks against oracles (energy drift,
//!   momentum conservation, …) and calls `std::process::exit(1)` on any
//!   failure. Its exit code IS the red/green signal.
//! - `demo` — illustration only; CI compile-checks it (via `grade`'s clippy)
//!   but never runs it, because it asserts nothing.
//!
//! # Why this command exists
//!
//! A `validator`'s oracle checks fire only when the binary is *run*. CI
//! compiles every example (workspace member) but the `tests-debug` /
//! `tests-release` shards — CI's only test-execution gate — intentionally
//! exclude examples (they carry no `#[test]`). So a validator's checks run
//! nowhere unless something runs the binary. This command is that something:
//! it discovers every crate declaring `example_kind = "validator"` from `cargo
//! metadata` and runs each in `--release`, failing if any exits non-zero.
//!
//! Discovery is by manifest marker, not a hand-maintained list, so a new
//! *marked* validator is picked up automatically — narrowing the drift class
//! where a crate silently falls out of CI coverage. Two residual holes are
//! held down explicitly: discovering zero validators is a hard error (a silent
//! green no-op would be the very failure this closes), and self-gating
//! examples missing the marker are audited (below).
//!
//! # The audit (drift backstop)
//!
//! Discovery is only airtight if every self-gating example actually carries
//! the marker. The command therefore also audits: any example whose source
//! calls `std::process::exit(1)` but does NOT declare `example_kind =
//! "validator"` is reported as an undeclared validator — its checks would run
//! nowhere. Today this is a warning that quantifies the gap; promoting it to
//! a hard failure is the fan-out step once every validator is marked. (The
//! substring scan is deliberately coarse; harden it before that promotion so
//! `exit(2)` / aliased-`exit` / `ExitCode::FAILURE` forms cannot slip past.)

use anyhow::{Context, Result};
use owo_colors::OwoColorize;
use std::path::{Path, PathBuf};
use xshell::{cmd, Shell};

/// A workspace crate paired with the facts the contract needs.
struct CrateEntry {
    /// Cargo package name (the `-p` target).
    name: String,
    /// Absolute crate directory (parent of its `Cargo.toml`).
    dir: PathBuf,
    /// The declared `example_kind`, if any.
    kind: Option<String>,
    /// Whether the crate lives under an `examples/` directory (scopes the
    /// undeclared-validator audit; discovery of validators is by marker
    /// alone, independent of path).
    is_example: bool,
}

/// The two `example_kind` values under `[package.metadata.cortenforge]`.
const KIND_VALIDATOR: &str = "validator";
const KIND_DEMO: &str = "demo";

/// Entry point for `cargo xtask run-validators`.
///
/// Discovers every declared `validator`, runs each red-or-green, and returns
/// an error (non-zero exit for CI) if any validator fails to pass. Undeclared
/// validators are surfaced as a warning but do not (yet) fail the gate.
pub fn run() -> Result<()> {
    let sh = Shell::new()?;
    let crates = discover_crates(&sh)?;

    warn_unknown_kinds(&crates);

    let validators: Vec<&CrateEntry> = crates
        .iter()
        .filter(|c| c.kind.as_deref() == Some(KIND_VALIDATOR))
        .collect();

    let undeclared: Vec<&CrateEntry> = crates
        .iter()
        .filter(|c| {
            c.is_example
                && c.kind.as_deref() != Some(KIND_VALIDATOR)
                && looks_like_validator(&c.dir)
        })
        .collect();

    println!(
        "{}",
        format!(
            "Validation contract — {} declared validator(s), {} undeclared",
            validators.len(),
            undeclared.len()
        )
        .bold()
    );

    audit_undeclared(&undeclared);

    // A discovery that finds zero validators would pass CI having run nothing
    // — the exact "passing CI ≠ tests ran" hole this command exists to close.
    // At least one example is marked once this ships, so zero means the marker
    // or a workspace member was dropped. Fail loudly rather than green-no-op.
    // (Printed after the audit so any dropped markers show up as undeclared.)
    if validators.is_empty() {
        anyhow::bail!(
            "no crate declares `example_kind = \"validator\"` — expected at least \
             one. A marker or workspace member was likely dropped; the validation \
             gate would otherwise pass having run nothing."
        );
    }

    println!("\nRunning validators (--release):\n");
    let mut failures: Vec<&str> = Vec::new();
    for v in &validators {
        let name = &v.name;
        println!("{} {}", "──▶".cyan(), name.bold());
        let outcome = cmd!(sh, "cargo run --release --quiet -p {name}").run();
        match outcome {
            Ok(()) => println!("  {} {}\n", "PASS".green().bold(), name),
            Err(_) => {
                println!("  {} {}\n", "FAIL".red().bold(), name);
                failures.push(name);
            }
        }
    }

    let total = validators.len();
    let passed = total - failures.len();
    println!("{}", "═".repeat(60));
    println!("  Validators: {passed}/{total} passed");
    if failures.is_empty() {
        println!("  {}", "ALL VALIDATORS GREEN".green().bold());
        Ok(())
    } else {
        println!("  {} {}", "RED:".red().bold(), failures.join(", "));
        anyhow::bail!("{} validator(s) failed", failures.len())
    }
}

/// Report undeclared validators. A warning today (quantifies the gap for
/// fan-out); the marker becomes mandatory once every validator carries it.
fn audit_undeclared(undeclared: &[&CrateEntry]) {
    if undeclared.is_empty() {
        return;
    }
    println!(
        "\n{} {} example(s) call `process::exit(1)` but do not declare \
         `example_kind = \"validator\"` — their checks run in NO CI context:",
        "warning:".yellow().bold(),
        undeclared.len()
    );
    for e in undeclared {
        println!("  - {}", e.name.yellow());
    }
}

/// Warn on any `example_kind` value outside the known vocabulary. A typo like
/// `"validater"` would otherwise silently downgrade a validator to non-run;
/// the `looks_like_validator` audit still catches it, but a named warning
/// points straight at the manifest.
fn warn_unknown_kinds(crates: &[CrateEntry]) {
    for c in crates {
        if let Some(kind) = &c.kind {
            if kind != KIND_VALIDATOR && kind != KIND_DEMO {
                println!(
                    "{} {} declares unknown example_kind = {kind:?} (expected \
                     {KIND_VALIDATOR:?} or {KIND_DEMO:?})",
                    "warning:".yellow().bold(),
                    c.name,
                );
            }
        }
    }
}

/// Read the workspace crates from `cargo metadata`, tagging each with its
/// declared `example_kind` and whether it lives under `examples/`.
///
/// `--no-deps` scopes packages to workspace members; each package's
/// `[package.metadata]` table is carried verbatim under `metadata`.
fn discover_crates(sh: &Shell) -> Result<Vec<CrateEntry>> {
    let json = cmd!(sh, "cargo metadata --format-version 1 --no-deps")
        .read()
        .context("Failed to run `cargo metadata`")?;
    let metadata: serde_json::Value =
        serde_json::from_str(&json).context("Failed to parse `cargo metadata` JSON")?;

    let packages = metadata["packages"]
        .as_array()
        .context("`cargo metadata`: missing 'packages' array")?;

    let mut crates = Vec::new();
    for pkg in packages {
        let manifest_path = pkg["manifest_path"]
            .as_str()
            .context("`cargo metadata`: package missing 'manifest_path'")?;
        let dir = Path::new(manifest_path)
            .parent()
            .context("manifest_path has no parent")?
            .to_path_buf();

        let is_example = dir.components().any(|c| c.as_os_str() == "examples");

        let name = pkg["name"]
            .as_str()
            .context("`cargo metadata`: package missing 'name'")?
            .to_string();
        let kind = pkg["metadata"]["cortenforge"]["example_kind"]
            .as_str()
            .map(String::from);

        crates.push(CrateEntry {
            name,
            dir,
            kind,
            is_example,
        });
    }
    Ok(crates)
}

/// Heuristic for the audit: does this crate's `src/` self-gate via a non-zero
/// exit? Presence of `std::process::exit(1)` (or `process::exit(1)`) marks a
/// binary that reports failure through its exit code.
fn looks_like_validator(dir: &Path) -> bool {
    let src = dir.join("src");
    for entry in walkdir::WalkDir::new(&src)
        .into_iter()
        .filter_map(Result::ok)
    {
        if entry.path().extension().is_some_and(|e| e == "rs") {
            if let Ok(text) = std::fs::read_to_string(entry.path()) {
                if text.contains("process::exit(1)") {
                    return true;
                }
            }
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::looks_like_validator;
    use std::fs;
    use std::path::PathBuf;

    /// Unique temp dir for one test case, isolated by name (no external deps).
    fn scratch(tag: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("xtask_validators_{tag}"));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(dir.join("src")).expect("create scratch src dir");
        dir
    }

    #[test]
    fn detects_exit_gate_in_source() {
        let dir = scratch("gate");
        fs::write(
            dir.join("src/main.rs"),
            "fn main() { if fail { std::process::exit(1); } }",
        )
        .expect("write source");
        assert!(looks_like_validator(&dir));
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn ignores_demo_without_exit_gate() {
        let dir = scratch("demo");
        fs::write(
            dir.join("src/main.rs"),
            "fn main() { println!(\"just a demonstration\"); }",
        )
        .expect("write source");
        assert!(!looks_like_validator(&dir));
        let _ = fs::remove_dir_all(&dir);
    }
}
