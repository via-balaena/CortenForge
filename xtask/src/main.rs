//! CortenForge Quality Enforcement Tool
//!
//! This is the "face tattoo" of the project - visible, permanent, defining.
//! All quality standards are enforced through this single entry point.
//!
//! # Commands
//!
//! - `cargo xtask check` - Run all quality checks
//! - `cargo xtask grade <crate>` - Grade a specific crate
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
//! See STANDARDS.md for full details.

mod check;
mod complete;
mod grade;
mod setup;

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

        /// Output format
        #[arg(long, default_value = "pretty")]
        format: String,
    },

    /// Record A-grade completion for a crate
    Complete {
        /// The crate to mark as complete
        #[arg(name = "CRATE")]
        crate_name: String,

        /// Skip interactive API review confirmation
        #[arg(long)]
        skip_review: bool,
    },

    /// Run full CI suite (same as GitHub Actions)
    Ci,

    /// List all crates and their current grades
    Status,

    /// Set up development environment (git hooks, verify tools)
    Setup,

    /// Remove git hooks installed by setup
    Uninstall,
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    match cli.command {
        Commands::Check { ci } => check::run(ci),
        Commands::Grade { crate_name, format } => grade::run(&crate_name, &format),
        Commands::Complete {
            crate_name,
            skip_review,
        } => complete::run(&crate_name, skip_review),
        Commands::Ci => check::run_ci(),
        Commands::Status => grade::status(),
        Commands::Setup => setup::run(),
        Commands::Uninstall => setup::uninstall(),
    }
}
