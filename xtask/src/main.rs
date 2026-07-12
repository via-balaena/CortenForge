//! CortenForge Quality Enforcement Tool
//!
//! This is the "face tattoo" of the project - visible, permanent, defining.
//! All quality standards are enforced through this single entry point.
//!
//! # Commands
//!
//! - `cargo xtask check` - Run all quality checks
//! - `cargo xtask grade <crate>` - Grade a specific crate
//! - `cargo xtask grade-all` - Grade every workspace crate (CI entry point)
//! - `cargo xtask complete <crate>` - Record A-grade completion
//! - `cargo xtask ci` - Full CI suite (same as GitHub Actions)
//!
//! # The A-Grade Standard
//!
//! Every crate must meet all seven criteria at A-grade before shipping:
//!
//! 1. Test Coverage ≥90%
//! 2. Documentation - Zero warnings
//! 3. Clippy - Zero warnings
//! 4. Safety - Zero unwrap/expect in library code
//! 5. Dependencies - Minimal, justified
//! 6. Bevy-free (Layer 0) - No bevy in dependency tree
//! 7. API Design - Idiomatic, intuitive (manual review)
//!
//! See docs/STANDARDS.md for full details.

mod affected;
mod check;
mod complete;
mod grade;
mod setup;
mod validators;

use anyhow::Result;
use clap::{Parser, Subcommand};

/// CortenForge Quality Enforcement
///
/// A-grade or it doesn't ship. No exceptions.
#[derive(Parser)]
#[command(name = "xtask")]
#[command(about = "Quality enforcement for CortenForge", long_about = None)]
#[command(version)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Run all quality checks across the workspace
    Check {
        /// Run in CI mode (stricter, fails on any issue)
        #[arg(long)]
        ci: bool,
    },

    /// Grade a specific crate against the A-grade standard
    Grade {
        /// The crate to grade (e.g., "mesh-types")
        #[arg(name = "CRATE")]
        crate_name: String,

        /// Suppress progress logging; show only the final report
        #[arg(long, conflicts_with = "verbose")]
        quiet: bool,

        /// Enable heartbeat during long-running stages (every 30s)
        #[arg(long, conflicts_with = "quiet")]
        verbose: bool,

        /// Emit the grade report as JSON instead of the Unicode table
        #[arg(long)]
        json: bool,

        /// Skip the Coverage criterion (reports N/A). Coverage runs
        /// cargo llvm-cov in release (5-10 min per crate) — too slow
        /// for per-PR CI. Dedicated coverage jobs run without the flag.
        #[arg(long)]
        skip_coverage: bool,
    },

    /// Grade every workspace crate and aggregate into a single summary
    GradeAll {
        /// Suppress per-crate progress; show only the final summary
        #[arg(long, conflicts_with = "verbose")]
        quiet: bool,

        /// Enable heartbeat during long-running stages (every 30s)
        #[arg(long, conflicts_with = "quiet")]
        verbose: bool,

        /// Skip the Coverage criterion across every crate. Required
        /// for CI use — cargo llvm-cov is too slow at workspace scale.
        #[arg(long)]
        skip_coverage: bool,

        /// Grade only a deterministic 1/N slice of the workspace, as
        /// `i/N` (1-based; e.g. `--shard 2/3`). Lets CI fan grade-all
        /// out across N parallel jobs to cut wall time. Crates are
        /// assigned round-robin over the sorted list so heavy crates
        /// scatter evenly across shards. Omit to grade everything.
        #[arg(long, value_parser = parse_shard)]
        shard: Option<(usize, usize)>,

        /// Restrict grading to a comma-separated set of crates (PR-scoped
        /// CI; pass the output of `cargo xtask affected`). Applied before
        /// `--shard`, so each shard grades a slice of this set. Omit to
        /// grade the full workspace (the gate on main/merge). An empty
        /// value (`--only ""`) grades nothing — the no-op for a PR that
        /// touches no crate.
        #[arg(long, value_delimiter = ',')]
        only: Option<Vec<String>>,
    },

    /// Record A-grade completion for a crate
    Complete {
        /// The crate to mark as complete
        #[arg(name = "CRATE")]
        crate_name: String,

        /// Bypass manual API review (records reviewer as "automated (forced)")
        #[arg(long)]
        force: bool,
    },

    /// List workspace crates affected by a diff (changed crates + their
    /// reverse-dependency closure), for PR-scoped CI.
    Affected {
        /// Git ref to diff against (the PR base); compared as `base...HEAD`.
        #[arg(long, default_value = "origin/main")]
        base: String,

        /// Emit `{"needs_full": bool, "crates": [...]}` JSON for CI to parse
        /// instead of a human-readable summary.
        #[arg(long)]
        json: bool,
    },

    /// Run full CI suite (same as GitHub Actions)
    Ci,

    /// List all crates and their current grades
    Status,

    /// Discover and run the fundamentals example-validators red-or-green.
    ///
    /// Runs every example declaring `[package.metadata.cortenforge]
    /// example_kind = "validator"` in `--release` and fails if any exits
    /// non-zero. Discovery is by manifest marker (no hand-maintained list),
    /// so a new validator is gated automatically. See `validators.rs`.
    RunValidators,

    /// Set up development environment (git hooks, verify tools)
    Setup,

    /// Remove git hooks installed by setup
    Uninstall,
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    match cli.command {
        Commands::Check { ci } => check::run(ci),
        Commands::Grade {
            crate_name,
            quiet,
            verbose,
            json,
            skip_coverage,
        } => grade::run(
            &crate_name,
            grade::Verbosity {
                quiet,
                verbose,
                json,
                skip_coverage,
            },
        ),
        Commands::GradeAll {
            quiet,
            verbose,
            skip_coverage,
            shard,
            only,
        } => grade::run_all(
            grade::Verbosity {
                quiet,
                verbose,
                json: false,
                skip_coverage,
            },
            shard,
            only,
        ),
        Commands::Complete { crate_name, force } => complete::run(&crate_name, force),
        Commands::Affected { base, json } => affected::run(&base, json),
        Commands::Ci => check::run_ci(),
        Commands::Status => grade::status(),
        Commands::RunValidators => validators::run(),
        Commands::Setup => setup::run(),
        Commands::Uninstall => setup::uninstall(),
    }
}

/// clap value parser for `--shard i/N` (1-based shard index over N shards).
///
/// Rejects malformed input, a zero shard count, and an index outside
/// `1..=N` so a typo fails fast at arg-parse time rather than silently
/// grading the wrong slice in CI.
fn parse_shard(s: &str) -> Result<(usize, usize), String> {
    let (i, n) = s
        .split_once('/')
        .ok_or_else(|| format!("expected `i/N`, got `{s}`"))?;
    let i: usize = i
        .trim()
        .parse()
        .map_err(|_| format!("shard index `{i}` is not a number"))?;
    let n: usize = n
        .trim()
        .parse()
        .map_err(|_| format!("shard count `{n}` is not a number"))?;
    if n == 0 {
        return Err("shard count must be ≥ 1".to_string());
    }
    if i < 1 || i > n {
        return Err(format!("shard index {i} out of range 1..={n}"));
    }
    Ok((i, n))
}

#[cfg(test)]
mod tests {
    use super::parse_shard;

    #[test]
    fn parse_shard_accepts_valid() {
        assert_eq!(parse_shard("1/3"), Ok((1, 3)));
        assert_eq!(parse_shard("3/3"), Ok((3, 3)));
        assert_eq!(parse_shard(" 2 / 4 "), Ok((2, 4)));
    }

    #[test]
    fn parse_shard_rejects_malformed() {
        assert!(parse_shard("2").is_err()); // missing /N
        assert!(parse_shard("x/3").is_err()); // non-numeric index
        assert!(parse_shard("1/y").is_err()); // non-numeric count
        assert!(parse_shard("0/3").is_err()); // index below 1
        assert!(parse_shard("4/3").is_err()); // index above count
        assert!(parse_shard("1/0").is_err()); // zero count
    }
}
