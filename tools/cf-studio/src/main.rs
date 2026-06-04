//! `cf-studio` binary — a thin `clap` dispatcher over the `cf-studio`
//! lib, where the per-command logic (and its tests) live.

use std::path::PathBuf;

use anyhow::Result;
use clap::{Parser, Subcommand};

/// CortenForge Studio — the guided scan→cast workflow, as a headless CLI.
/// Each command runs one step and autosaves, so the workflow resumes
/// across invocations.
#[derive(Parser)]
#[command(name = "cf-studio", version)]
struct Cli {
    /// Path to the project file (created by `new`, updated by each step).
    #[arg(long, global = true, default_value = "studio-project.json")]
    project: PathBuf,
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Create a new project.
    New {
        /// A name for the project.
        name: String,
    },
    /// Show the current step + progress.
    Status,
    /// Step 1 — add a scan (validates it loads + has geometry).
    Scan {
        /// Path to the scan file (STL / OBJ / PLY).
        scan_file: PathBuf,
    },
    /// Step 2 — accept a cleaned scan + its `.prep.toml`.
    Prep {
        /// The cleaned, watertight STL.
        cleaned_stl: PathBuf,
        /// The `.prep.toml` (cap planes + centerline).
        prep_toml: PathBuf,
    },
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Command::New { name } => cf_studio::cmd_new(&cli.project, &name),
        Command::Status => cf_studio::cmd_status(&cli.project),
        Command::Scan { scan_file } => cf_studio::cmd_scan(&cli.project, &scan_file),
        Command::Prep {
            cleaned_stl,
            prep_toml,
        } => cf_studio::cmd_prep(&cli.project, &cleaned_stl, &prep_toml),
    }
}
