//! Command-line interface: resolve the two vertebra STL paths from `--dir` (by
//! FMA id) or explicit `--l4` / `--l5` flags. The disc is no longer an input —
//! it is painted + lofted in the Design state.

use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::Parser;

/// Paint the L4–L5 endplates, loft the disc, and simulate the coupled FSU.
#[derive(Parser)]
#[command(name = "cf-spine-studio")]
pub(crate) struct Cli {
    /// Directory holding the two BodyParts3D vertebra STLs, named by FMA id
    /// (FMA13075 = L4, FMA13076 = L5). Native mm.
    #[arg(long)]
    dir: Option<PathBuf>,
    /// Explicit L4 STL path (overrides `--dir`).
    #[arg(long)]
    l4: Option<PathBuf>,
    /// Explicit L5 STL path (overrides `--dir`).
    #[arg(long)]
    l5: Option<PathBuf>,
}

/// Resolve the two vertebra STL paths from `--dir` (by FMA id) or explicit flags.
pub(crate) fn resolve_paths(cli: &Cli) -> Result<(PathBuf, PathBuf)> {
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
    Ok((l4, l5))
}
