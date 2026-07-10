//! Command-line interface: resolve the three input STL paths from `--dir` (by
//! FMA id) or explicit `--l4` / `--l5` / `--disc` flags.

use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::Parser;

/// Render the assembled L4–L5 FSU flexing in a native Bevy window.
#[derive(Parser)]
#[command(name = "cf-spine-viewer")]
pub(crate) struct Cli {
    /// Directory holding the three BodyParts3D STLs, named by FMA id
    /// (FMA13075 = L4, FMA13076 = L5, FMA16036 = disc). Native mm.
    #[arg(long)]
    dir: Option<PathBuf>,
    /// Explicit L4 STL path (overrides `--dir`).
    #[arg(long)]
    l4: Option<PathBuf>,
    /// Explicit L5 STL path (overrides `--dir`).
    #[arg(long)]
    l5: Option<PathBuf>,
    /// Explicit disc STL path (overrides `--dir`).
    #[arg(long)]
    disc: Option<PathBuf>,
}

/// Resolve the three STL paths from `--dir` (by FMA id) or explicit flags.
pub(crate) fn resolve_paths(cli: &Cli) -> Result<(PathBuf, PathBuf, PathBuf)> {
    let from_dir = |id: &str| cli.dir.as_ref().map(|d| d.join(format!("FMA{id}.stl")));
    let l4 = cli
        .l4
        .clone()
        .or_else(|| from_dir("13075"))
        .context("provide --l4 <path> or --dir <dir with FMA13075.stl>")?;
    let l5 = cli
        .l5
        .clone()
        .or_else(|| from_dir("13076"))
        .context("provide --l5 <path> or --dir <dir with FMA13076.stl>")?;
    let disc = cli
        .disc
        .clone()
        .or_else(|| from_dir("16036"))
        .context("provide --disc <path> or --dir <dir with FMA16036.stl>")?;
    Ok((l4, l5, disc))
}
