//! Discover and run the example-validators red-or-green.
//!
//! # The validation contract
//!
//! Every **headless** `examples/**` crate must declare a role in its
//! `Cargo.toml` under `[package.metadata.cortenforge] example_kind`:
//!
//! - `validator` — the binary self-checks against oracles (energy drift,
//!   momentum conservation, a closed-form stress, …) and terminates with a
//!   **non-zero exit** on any failure. Its exit code IS the red/green signal.
//! - `demo` — illustration only; CI compile-checks it (via `grade`'s clippy)
//!   but never runs it, because it asserts nothing.
//!
//! "Headless" means the crate has no direct Bevy dependency. Bevy examples are
//! *visual* — their axis is a rendered window a human inspects, which CI cannot
//! run — so they are exempt from this contract and are compile-only by design.
//!
//! # Why the signal is the exit code, not a fixed idiom
//!
//! A validator may signal failure two ways, and the contract deliberately does
//! not care which: an explicit `std::process::exit(1)`, or a panicking
//! assertion (`assert!`, `assert_eq!`, `assert_relative_eq!`, …) — a panic in
//! `main` aborts with a non-zero code (101) exactly like `exit(1)`. This
//! command runs each validator and reads its exit code, so **both conventions
//! are handled identically**. There is nothing to detect at run time; the
//! marker declares intent and the exit code reports the result.
//!
//! # Why this command exists
//!
//! A validator's oracle checks fire only when the binary is *run*. CI compiles
//! every example (workspace member) but the `tests-debug` / `tests-release`
//! shards — CI's only test-execution gate — intentionally exclude examples
//! (they carry no `#[test]`). So a validator's checks run nowhere unless
//! something runs the binary. This command is that something: it discovers
//! every crate declaring `example_kind = "validator"` from `cargo metadata`
//! and runs each in `--release`, failing if any exits non-zero.
//!
//! Discovery is by manifest marker, not a hand-maintained list, so a new
//! *marked* validator is picked up automatically. Discovering zero validators
//! is a hard error — a silent green no-op would be the very "passing CI ≠ tests
//! ran" failure this closes.
//!
//! # The classification audit (drift backstop)
//!
//! Discovery is only airtight if every headless example actually declares a
//! kind. The command therefore audits the contract and **hard-fails** on either
//! gap (checked before the validator runs, so a metadata-only violation fails
//! fast):
//!
//! - **Unclassified** — a headless example with no valid `example_kind`
//!   (missing, or an unrecognised value). Whether its source self-gates is
//!   shown as a triage hint (`validator?` / `demo?`).
//! - **Misclassified** — a crate declared `demo` whose source self-gates; its
//!   oracle checks would run in no CI context.
//!
//! The self-gating heuristic (`source_self_gates`) is a *hint for
//! classification only*, never the gate — the marker is the source of truth, so
//! the heuristic never needs to recognise every possible failure idiom.
//!
//! Scope note: only *headless* examples are audited. A dual-mode crate — one
//! that runs a headless oracle by default but also offers an opt-in Bevy view
//! (`CF_VISUAL=1`) — carries a Bevy dep and so reads as non-headless, leaving
//! its oracle outside this audit. The intended resolution is to split such a
//! crate into a headless validator and a separate Bevy demo, not to widen the
//! audit to guess at Bevy crates' intent.

use crate::pr_scope::{filter_only, select_shard_weighted};
use anyhow::{Context, Result};
use owo_colors::OwoColorize;
use regex::Regex;
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::LazyLock;
use std::time::Instant;
use xshell::{cmd, Shell};

/// Committed per-validator wall-time baseline (seconds), the weight source for
/// shard balancing. Regenerate with `run-validators --record-timings`.
const TIMINGS_PATH: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/validator_timings.json");

/// Weight (seconds) for a validator absent from the timings baseline — e.g. one
/// added since the last `--record-timings` refresh. Deliberately generous so an
/// unknown validator is treated as heavy and isolated into its own bucket by the
/// LPT split rather than risking a slow cluster; it self-corrects on the next
/// refresh.
const UNKNOWN_VALIDATOR_SECS: f64 = 60.0;

/// A workspace crate paired with the facts the contract needs.
struct CrateEntry {
    /// Cargo package name (the `-p` target).
    name: String,
    /// Absolute crate directory (parent of its `Cargo.toml`).
    dir: PathBuf,
    /// The declared `example_kind`, if any.
    kind: Option<String>,
    /// Whether the crate lives under an `examples/` directory (scopes the
    /// classification contract; discovery of validators is by marker alone,
    /// independent of path).
    is_example: bool,
    /// Whether the crate has no direct Bevy dependency. Headless examples are
    /// bound by the classification contract; Bevy (visual) examples are exempt.
    is_headless: bool,
}

/// The two `example_kind` values under `[package.metadata.cortenforge]`.
const KIND_VALIDATOR: &str = "validator";
const KIND_DEMO: &str = "demo";

/// A contract gap found while auditing one crate.
enum Gap {
    /// Headless example carrying no valid `example_kind` (missing or
    /// unrecognised) — must be classified. `self_gates` is the triage hint:
    /// `true` ⇒ likely `validator`.
    Unclassified { self_gates: bool },
    /// Declared `demo` yet the source self-gates — a likely mis-marked
    /// validator whose oracle checks would run nowhere.
    Misclassified,
}

/// Entry point for `cargo xtask run-validators`.
///
/// Discovers every declared `validator`, runs each red-or-green, and returns an
/// error (non-zero exit for CI) if any validator fails to pass. Contract gaps
/// (unclassified / misclassified headless examples) also fail the gate — a hard
/// error, checked before the validator runs so a metadata-only violation fails
/// fast.
///
/// # PR-scoping the run (not the audit)
///
/// `only` (from `--only`, the affected set) and `shard` (from `--shard i/N`)
/// narrow **only the run step**, mirroring `grade-all`: they select which
/// validators' binaries actually build+run, so a docs-only PR runs none and a
/// foundational PR fans its affected validators across N parallel jobs. The
/// classification audit and the zero-discovered guard always cover the whole
/// workspace — the contract is the drift backstop and must never be narrowed.
/// A run set that is empty *after* filtering is a legitimate no-op (this shard
/// has no affected validator), distinct from the "zero discovered" hard-fail.
///
/// Shards are balanced by cost, not validator count: each validator's weight is
/// its measured runtime (the committed `validator_timings.json` baseline) plus a
/// per-crate compile constant ([`COMPILE_OVERHEAD_SECS`]). Runtime alone
/// clusters the few slow validators onto one shard; count alone ignores that a
/// handful run 100× longer — the sum balances both. With `record_timings`, each
/// validator's observed runtime is written back to the baseline after the run.
pub fn run(
    only: Option<Vec<String>>,
    shard: Option<(usize, usize)>,
    record_timings_flag: bool,
) -> Result<()> {
    let sh = Shell::new()?;
    let crates = discover_crates(&sh)?;

    warn_unknown_kinds(&crates);

    let validators: Vec<&CrateEntry> = crates
        .iter()
        .filter(|c| c.kind.as_deref() == Some(KIND_VALIDATOR))
        .collect();

    // Audit the classification contract. `source_self_gates` is only consulted
    // for in-scope crates that are not already declared validators — it is a
    // triage hint, never the gate.
    let mut unclassified: Vec<(&CrateEntry, bool)> = Vec::new();
    let mut misclassified: Vec<&CrateEntry> = Vec::new();
    for c in &crates {
        let self_gates = c.is_example
            && c.is_headless
            && c.kind.as_deref() != Some(KIND_VALIDATOR)
            && source_self_gates(&c.dir);
        match audit_gap(c.is_example, c.is_headless, c.kind.as_deref(), self_gates) {
            Some(Gap::Unclassified { self_gates }) => unclassified.push((c, self_gates)),
            Some(Gap::Misclassified) => misclassified.push(c),
            None => {}
        }
    }

    println!(
        "{}",
        format!(
            "Validation contract — {} declared validator(s), {} unclassified headless \
             example(s), {} possibly-misclassified",
            validators.len(),
            unclassified.len(),
            misclassified.len()
        )
        .bold()
    );

    audit_unclassified(&unclassified);
    audit_misclassified(&misclassified);

    // HARD-FAIL on any classification gap (arc-close b rung 8). The contract is
    // enforced now that every headless example is classified: an unclassified or
    // misclassified headless example fails the gate instead of merely warning, so
    // the 0/0 state cannot silently regress. Checked before the heavy validator
    // runs — a metadata-only violation fails fast. `source_self_gates` remains a
    // triage hint for the report, never the gate (the marker is the source of
    // truth); the gate is purely "is the crate classified".
    if let Some(msg) = contract_violation(unclassified.len(), misclassified.len()) {
        anyhow::bail!("{msg}");
    }

    // A discovery that finds zero validators would pass CI having run nothing
    // — the exact "passing CI ≠ tests ran" hole this command exists to close.
    // At least one example is marked, so zero means the marker or a workspace
    // member was dropped. Fail loudly rather than green-no-op. (Printed after
    // the audit so any dropped markers show up as unclassified.)
    if validators.is_empty() {
        anyhow::bail!(
            "no crate declares `example_kind = \"validator\"` — expected at least \
             one. A marker or workspace member was likely dropped; the validation \
             gate would otherwise pass having run nothing."
        );
    }

    // PR-scope the RUN only (the audit above stays whole-workspace): apply
    // `--only` (the affected set), then split by `--shard` via weight-aware LPT
    // bin-packing on measured wall-time. An empty result means "no affected
    // validator in this selection" — a legitimate no-op, NOT the zero-discovered
    // hard-fail (that guard ran on the full set above).
    let timings = load_timings();
    let mut validator_names: Vec<String> = validators.iter().map(|v| v.name.clone()).collect();
    validator_names.sort();
    let weighted: Vec<(String, u64)> = filter_only(&validator_names, only.as_deref())
        .into_iter()
        .map(|name| {
            let w = weight_millis(&timings, &name);
            (name, w)
        })
        .collect();
    let run_names = select_shard_weighted(&weighted, shard);

    if run_names.is_empty() {
        println!(
            "\n{} no validator in this selection (--only / --shard) — nothing to run.",
            "note:".bold()
        );
        return Ok(());
    }

    println!("\nRunning {} validator(s) (--release):\n", run_names.len());
    let mut failures: Vec<&str> = Vec::new();
    let mut measured: HashMap<String, f64> = HashMap::new();
    for name in &run_names {
        println!("{} {}", "──▶".cyan(), name.bold());
        let start = Instant::now();
        let outcome = cmd!(sh, "cargo run --release --quiet -p {name}").run();
        let secs = start.elapsed().as_secs_f64();
        measured.insert(name.clone(), secs);
        match outcome {
            Ok(()) => println!("  {} {name} ({secs:.1}s)\n", "PASS".green().bold()),
            Err(_) => {
                println!("  {} {name} ({secs:.1}s)\n", "FAIL".red().bold());
                failures.push(name.as_str());
            }
        }
    }

    if record_timings_flag {
        record_timings(&timings, &measured)?;
        println!(
            "  {} {} timing(s) → {TIMINGS_PATH}",
            "recorded".cyan().bold(),
            measured.len()
        );
    }

    let total = run_names.len();
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

/// Classify one crate against the contract. Pure (no fs/cargo) so the rule is
/// unit-testable: only headless examples are in scope; a Bevy example or a
/// non-example yields no gap.
fn audit_gap(
    is_example: bool,
    is_headless: bool,
    kind: Option<&str>,
    self_gates: bool,
) -> Option<Gap> {
    if !is_example || !is_headless {
        return None;
    }
    match kind {
        Some(KIND_VALIDATOR) => None,
        Some(KIND_DEMO) if self_gates => Some(Gap::Misclassified),
        Some(KIND_DEMO) => None,
        // `None` (no marker) or an unknown/typo'd kind: both leave the crate
        // un-run, so both are contract gaps. (`warn_unknown_kinds` additionally
        // names the typo so the manifest is easy to find.)
        _ => Some(Gap::Unclassified { self_gates }),
    }
}

/// The enforcement policy over the audited gap counts. Pure (no fs/cargo) so the
/// rule is unit-testable, mirroring `audit_gap`. Returns `Some(message)` — the
/// hard-fail reason — when the contract is violated (any unclassified or
/// misclassified headless example), or `None` when the contract holds. The gate
/// is deliberately count-only: `audit_gap` already decided *what* is a gap; this
/// decides *whether any gap fails the run*.
fn contract_violation(unclassified: usize, misclassified: usize) -> Option<String> {
    if unclassified == 0 && misclassified == 0 {
        return None;
    }
    Some(format!(
        "validation contract violated: {unclassified} unclassified + {misclassified} \
         possibly-misclassified headless example(s) (see the audit above). Every headless \
         example must declare `example_kind = \"{KIND_VALIDATOR}\"` or \"{KIND_DEMO}\", and a \
         `{KIND_DEMO}` must not self-gate. Add the marker to the crate's \
         `[package.metadata.cortenforge]` table."
    ))
}

/// Report unclassified headless examples — the fan-out worklist. Each is
/// tagged with the triage hint so classification is a glance, not a code-read.
fn audit_unclassified(unclassified: &[(&CrateEntry, bool)]) {
    if unclassified.is_empty() {
        return;
    }
    println!(
        "\n{} {} headless example(s) declare no `example_kind` — every headless \
         example must be classified `validator` or `demo`:",
        "error:".red().bold(),
        unclassified.len()
    );
    for (e, self_gates) in unclassified {
        if *self_gates {
            println!("  - {} ({})", e.name.yellow(), "validator?".green());
        } else {
            println!("  - {} ({})", e.name.yellow(), "demo?".blue());
        }
    }
}

/// Report `demo`-declared crates whose source self-gates — a likely mis-marked
/// validator whose oracle checks run in no CI context.
fn audit_misclassified(misclassified: &[&CrateEntry]) {
    if misclassified.is_empty() {
        return;
    }
    println!(
        "\n{} {} example(s) declare `example_kind = \"demo\"` but self-gate — likely \
         mis-marked validators (their checks run nowhere):",
        "error:".red().bold(),
        misclassified.len()
    );
    for e in misclassified {
        println!("  - {}", e.name.yellow());
    }
}

/// Warn on any `example_kind` value outside the known vocabulary. A typo like
/// `"validater"` would otherwise silently downgrade a validator to non-run.
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
/// declared `example_kind`, whether it lives under `examples/`, and whether it
/// is headless (no direct Bevy dependency).
///
/// `--no-deps` scopes packages to workspace members; each package's
/// `[package.metadata]` table is carried verbatim under `metadata`, and its
/// direct dependencies under `dependencies`.
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

        let is_headless = !depends_on_bevy(pkg);

        crates.push(CrateEntry {
            name,
            dir,
            kind,
            is_example,
            is_headless,
        });
    }
    Ok(crates)
}

/// Does a `cargo metadata` package have any direct Bevy dependency? Covers the
/// whole family (`bevy`, `sim-bevy`, `sim-bevy-soft`, `cf-bevy-common`,
/// `sim-ml-chassis-bevy`, …) by matching the substring `bevy` in the dep name.
fn depends_on_bevy(pkg: &serde_json::Value) -> bool {
    pkg["dependencies"].as_array().is_some_and(|deps| {
        deps.iter()
            .any(|d| d["name"].as_str().is_some_and(|n| n.contains("bevy")))
    })
}

/// Load the committed per-validator wall-time baseline (seconds), or an empty
/// map if the file is missing or unparseable. An empty map is graceful: every
/// validator then falls back to [`UNKNOWN_VALIDATOR_SECS`], so sharding degrades
/// to a uniform (round-robin-equivalent) split until the baseline is recorded.
fn load_timings() -> HashMap<String, f64> {
    std::fs::read_to_string(TIMINGS_PATH)
        .ok()
        .map(|json| parse_timings(&json))
        .unwrap_or_default()
}

/// Parse a `{ "validator-name": seconds }` JSON map; a malformed file yields an
/// empty map (a bad baseline must never crash the gate).
fn parse_timings(json: &str) -> HashMap<String, f64> {
    serde_json::from_str(json).unwrap_or_default()
}

/// Fixed per-validator compile cost (seconds) added to every validator's
/// measured runtime to form its shard weight.
///
/// The recorded timings are pure *runtime* (warm cache), but in CI each
/// validator ALSO costs its own example-crate compile — roughly uniform per
/// crate and independent of runtime. Balancing runtime alone (an earlier
/// iteration) piled the many fast validators onto one shard whose *compile* then
/// dominated its wall-time; adding a per-crate constant makes LPT spread them.
/// Back-solved from CI wall-times at ~3.8 s/crate (the shared-workspace-tree
/// compile is a fixed per-shard cost paid by every shard, so it is deliberately
/// NOT attributed here). Approximate by design — refine if CI shard times stay
/// skewed.
const COMPILE_OVERHEAD_SECS: f64 = 3.8;

/// The shard weight (integer milliseconds) for a validator: its measured runtime
/// (or [`UNKNOWN_VALIDATOR_SECS`] when absent) plus [`COMPILE_OVERHEAD_SECS`], so
/// the LPT split balances BOTH the few long-running validators and the compile
/// cost of the many fast ones. Milliseconds keep the bin-packer's `u64` exact.
fn weight_millis(timings: &HashMap<String, f64>, name: &str) -> u64 {
    let runtime = timings.get(name).copied().unwrap_or(UNKNOWN_VALIDATOR_SECS);
    ((runtime + COMPILE_OVERHEAD_SECS) * 1000.0) as u64
}

/// Merge freshly-measured timings into the baseline and write it back, sorted by
/// key so the committed file has a stable, reviewable diff. Existing entries not
/// re-measured this run (e.g. under `--shard`/`--only`) are preserved.
fn record_timings(existing: &HashMap<String, f64>, measured: &HashMap<String, f64>) -> Result<()> {
    let mut merged = existing.clone();
    for (name, secs) in measured {
        merged.insert(name.clone(), *secs);
    }
    let mut sorted: Vec<(&String, &f64)> = merged.iter().collect();
    sorted.sort_by(|a, b| a.0.cmp(b.0));
    let map: serde_json::Map<String, serde_json::Value> = sorted
        .into_iter()
        .map(|(k, v)| {
            (
                k.clone(),
                serde_json::json!((v * 1000.0).round() / 1000.0), // 3-dp, stable
            )
        })
        .collect();
    let json = serde_json::to_string_pretty(&serde_json::Value::Object(map))
        .context("serialize validator timings")?;
    std::fs::write(TIMINGS_PATH, json + "\n")
        .with_context(|| format!("write timings baseline to {TIMINGS_PATH}"))?;
    Ok(())
}

/// Triage heuristic: does this crate's `src/` self-gate — fail via a non-zero
/// exit? True if any source (with line comments stripped, so a mention in a
/// `//`/`//!` comment does not count) calls `process::exit`/`exit` with a
/// non-zero argument, or uses a panicking `assert*!` macro (a panic aborts
/// `main` with a non-zero code). A hint for classification only, never the gate
/// — so it need not recognise every idiom (`.unwrap()` / `-> Result` `?` gating
/// are missed; the marker, not this heuristic, is the source of truth).
fn source_self_gates(dir: &Path) -> bool {
    // Compiled once, not per crate — the audit calls this for every in-scope
    // example. `exit(<nonzero>)` or a panicking `assert*!` marks a self-gate.
    static FAILING_EXIT: LazyLock<Regex> =
        LazyLock::new(|| Regex::new(r"\bexit\s*\(\s*[^0\s)]").expect("valid regex"));
    static ASSERT_MACRO: LazyLock<Regex> =
        LazyLock::new(|| Regex::new(r"\bassert(_\w+)?\s*!").expect("valid regex"));

    let src = dir.join("src");
    for entry in walkdir::WalkDir::new(&src)
        .into_iter()
        .filter_map(Result::ok)
    {
        if entry.path().extension().is_some_and(|e| e == "rs") {
            if let Ok(text) = std::fs::read_to_string(entry.path()) {
                let code = strip_line_comments(&text);
                if FAILING_EXIT.is_match(&code) || ASSERT_MACRO.is_match(&code) {
                    return true;
                }
            }
        }
    }
    false
}

/// Drop `//`-to-end-of-line content so an `assert!`/`exit(1)` mentioned in a
/// line or doc comment does not read as self-gating. Coarse (does not parse
/// block comments or string literals), which is fine for a heuristic hint.
fn strip_line_comments(text: &str) -> String {
    text.lines()
        .map(|line| match line.find("//") {
            Some(i) => &line[..i],
            None => line,
        })
        .collect::<Vec<_>>()
        .join("\n")
}

#[cfg(test)]
mod tests {
    use super::{
        audit_gap, contract_violation, parse_timings, source_self_gates, weight_millis, Gap,
        COMPILE_OVERHEAD_SECS, KIND_DEMO, KIND_VALIDATOR, UNKNOWN_VALIDATOR_SECS,
    };
    use std::fs;
    use std::path::PathBuf;

    #[test]
    fn parse_timings_reads_name_to_seconds_map() {
        let t = parse_timings(r#"{"example-a": 12.5, "example-b": 3.0}"#);
        assert_eq!(t.get("example-a"), Some(&12.5));
        assert_eq!(t.get("example-b"), Some(&3.0));
    }

    #[test]
    fn parse_timings_tolerates_garbage() {
        // A malformed baseline must degrade to "no data", never crash the gate.
        assert!(parse_timings("not json").is_empty());
        assert!(parse_timings("").is_empty());
    }

    #[test]
    fn weight_millis_adds_compile_overhead_to_runtime_then_falls_back() {
        let t = parse_timings(r#"{"known": 4.2}"#);
        // recorded runtime + the per-crate compile constant.
        assert_eq!(
            weight_millis(&t, "known"),
            ((4.2 + COMPILE_OVERHEAD_SECS) * 1000.0) as u64
        );
        // Unknown → generous runtime placeholder (+ overhead) so LPT isolates it.
        assert_eq!(
            weight_millis(&t, "brand-new"),
            ((UNKNOWN_VALIDATOR_SECS + COMPILE_OVERHEAD_SECS) * 1000.0) as u64
        );
    }

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
        assert!(source_self_gates(&dir));
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn detects_assert_gate_in_source() {
        // The assert convention (~44 headless validators) the exit-only detector
        // missed — panics abort main with a non-zero code, so these self-gate.
        for macro_call in [
            "assert!(x > 0.0);",
            "assert_eq!(a, b);",
            "assert_relative_eq!(observed, analytic, max_relative = 1e-12);",
        ] {
            let dir = scratch("assert");
            fs::write(
                dir.join("src/main.rs"),
                format!("fn main() {{ {macro_call} }}"),
            )
            .expect("write source");
            assert!(source_self_gates(&dir), "should self-gate: {macro_call}");
            let _ = fs::remove_dir_all(&dir);
        }
    }

    #[test]
    fn ignores_assert_mentioned_only_in_a_comment() {
        // A doc/line comment that names `assert!`/`exit(1)` must not read as a
        // gate — line comments are stripped before matching.
        let dir = scratch("comment");
        fs::write(
            dir.join("src/main.rs"),
            "//! A demo. A real validator would assert_eq!(a, b) or exit(1).\n\
             fn main() { println!(\"demo\"); } // no assert!(x) here either",
        )
        .expect("write source");
        assert!(!source_self_gates(&dir));
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn audit_flags_unknown_kind_as_unclassified() {
        // A typo'd kind is neither run (not `validator`) nor a valid `demo`, so
        // it leaves the crate un-run — a contract gap, not a silent pass.
        assert!(
            matches!(
                audit_gap(true, true, Some("validater"), false),
                Some(Gap::Unclassified { .. })
            ),
            "unknown example_kind on a headless example is a gap"
        );
    }

    #[test]
    fn ignores_demo_without_gate() {
        let dir = scratch("demo");
        fs::write(
            dir.join("src/main.rs"),
            // A success-only `exit(0)` and the bare word \"assertion\" must NOT
            // read as self-gating.
            "fn main() { println!(\"just an assertion-free demonstration\"); std::process::exit(0); }",
        )
        .expect("write source");
        assert!(!source_self_gates(&dir));
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn audit_scopes_to_headless_examples() {
        // Non-example and Bevy-example crates are out of contract scope.
        assert!(audit_gap(false, true, None, true).is_none(), "non-example");
        assert!(
            audit_gap(true, false, None, true).is_none(),
            "bevy example exempt"
        );
    }

    #[test]
    fn audit_flags_unclassified_headless() {
        match audit_gap(true, true, None, true) {
            Some(Gap::Unclassified { self_gates }) => assert!(self_gates),
            _ => panic!("headless example with no kind should be Unclassified"),
        }
    }

    #[test]
    fn audit_flags_demo_that_self_gates() {
        assert!(
            matches!(
                audit_gap(true, true, Some(KIND_DEMO), true),
                Some(Gap::Misclassified)
            ),
            "demo that self-gates is a likely mis-marked validator"
        );
    }

    #[test]
    fn audit_passes_declared_crates() {
        assert!(
            audit_gap(true, true, Some(KIND_VALIDATOR), true).is_none(),
            "declared validator is fine"
        );
        assert!(
            audit_gap(true, true, Some(KIND_DEMO), false).is_none(),
            "demo that does not self-gate is fine"
        );
    }

    #[test]
    fn contract_holds_only_when_both_gap_counts_are_zero() {
        // The rung-8 enforcement policy: a fully-classified fleet (0/0) passes;
        // any unclassified OR misclassified example is a hard-fail.
        assert!(
            contract_violation(0, 0).is_none(),
            "0 unclassified / 0 misclassified is the passing contract state"
        );
        assert!(
            contract_violation(1, 0).is_some(),
            "an unclassified headless example must fail the gate"
        );
        assert!(
            contract_violation(0, 1).is_some(),
            "a misclassified headless example must fail the gate"
        );
        assert!(
            contract_violation(3, 2).is_some(),
            "both gap kinds present must fail the gate"
        );
    }

    #[test]
    fn contract_violation_message_names_both_counts() {
        // The fail message must carry the actionable numbers so CI logs point
        // straight at the scope of the gap.
        let msg = contract_violation(2, 1).expect("nonzero gaps violate the contract");
        assert!(msg.contains('2') && msg.contains('1'), "message: {msg}");
        assert!(
            msg.contains(KIND_VALIDATOR) && msg.contains(KIND_DEMO),
            "message should name both valid kinds: {msg}"
        );
    }
}
