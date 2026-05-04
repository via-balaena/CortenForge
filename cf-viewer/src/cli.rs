//! `clap`-derived CLI surface for `cf-view`.
//!
//! Three flags layered on top of the runtime side-panel dropdowns
//! (commit 5). The CLI seeds the initial state; the dropdowns then act
//! as runtime overrides on top of the seed:
//!
//! - `--scalar=<name>` validates against the loaded PLY's `scalar_names`
//!   (per-vertex extras keys); errors with the available names listed
//!   when the requested scalar is missing.
//! - `--colormap=<auto|divergent|sequential|categorical>` maps directly
//!   to [`ColormapOverride`]. `auto` defers to the value-distribution
//!   detector (Q5 lock).
//! - `--up=<+X|+Y|+Z>` selects [`UpAxis`]; `+Z` is the workspace default
//!   (mesh-v1.0 build-plate convention).
//!
//! See `docs/VIEWER_DESIGN.md` PR-shape commit 6 for the locked surface.

use std::path::PathBuf;

use anyhow::{Result, bail};
use clap::{Parser, ValueEnum};

use crate::UpAxis;
use crate::ui::{ColormapOverride, Selection};

/// `cf-view` CLI surface — one positional path + three optional flags.
#[derive(Parser, Debug)]
#[command(
    name = "cf-view",
    version,
    about = "CortenForge unified visual-review viewer",
    long_about = None,
)]
pub struct Cli {
    /// Path to the PLY file to view.
    pub path: PathBuf,

    /// Pre-select a per-vertex scalar by name. The name must match a key
    /// in the PLY's `extras` (per-vertex). When omitted, alphabetical
    /// first-pick applies (Q4 lock). Runtime-overridable via the side-
    /// panel scalar dropdown.
    #[arg(long)]
    pub scalar: Option<String>,

    /// Pre-select the colormap kind. `auto` defers to the Q5 distribution
    /// detector. Runtime-overridable via the side-panel colormap dropdown.
    #[arg(long, value_enum, default_value = "auto")]
    pub colormap: ColormapArg,

    /// Which input axis is up. `+Z` is the workspace default (mesh-v1.0
    /// build-plate convention; matches the legacy `f3d --up=+Z`). Not
    /// runtime-overridable — geometry is mapped to Bevy-Y-up at load.
    #[arg(long, value_enum, default_value = "+Z")]
    pub up: UpAxisArg,
}

/// CLI-side mirror of [`ColormapOverride`]. Keeps the `clap`-derived
/// surface independent of the UI-side enum so the wire format
/// (`auto`/`divergent`/...) is locked here, not implicitly through
/// `Display` on the UI type.
#[derive(ValueEnum, Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ColormapArg {
    /// Defer to the Q5 distribution detector.
    #[default]
    Auto,
    /// Force divergent (`coolwarm`-style bwr).
    Divergent,
    /// Force sequential (`viridis`).
    Sequential,
    /// Force categorical (`tab10`, indexed by `value mod 10`).
    Categorical,
}

impl From<ColormapArg> for ColormapOverride {
    fn from(c: ColormapArg) -> Self {
        match c {
            ColormapArg::Auto => Self::Auto,
            ColormapArg::Divergent => Self::Divergent,
            ColormapArg::Sequential => Self::Sequential,
            ColormapArg::Categorical => Self::Categorical,
        }
    }
}

/// CLI-side mirror of [`UpAxis`]. Variants render as `+X` / `+Y` / `+Z`
/// in `--help` and accept those exact strings.
#[derive(ValueEnum, Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum UpAxisArg {
    /// Input X axis is up — see [`UpAxis::PlusX`].
    #[value(name = "+X")]
    PlusX,
    /// Input Y axis is up — see [`UpAxis::PlusY`].
    #[value(name = "+Y")]
    PlusY,
    /// Input Z axis is up — see [`UpAxis::PlusZ`]. Workspace default.
    #[default]
    #[value(name = "+Z")]
    PlusZ,
}

impl From<UpAxisArg> for UpAxis {
    fn from(a: UpAxisArg) -> Self {
        match a {
            UpAxisArg::PlusX => Self::PlusX,
            UpAxisArg::PlusY => Self::PlusY,
            UpAxisArg::PlusZ => Self::PlusZ,
        }
    }
}

/// Build the initial [`Selection`] from CLI flags + the loaded PLY's
/// `scalar_names`.
///
/// `--scalar=<name>` must resolve to an index in `scalar_names`; otherwise
/// an `anyhow` error is returned listing the available names. When `scalar`
/// is `None`, the Q4 alphabetical first-pick (`scalar_index = 0`) applies.
///
/// # Errors
///
/// Returns an error when `--scalar=<name>` is supplied and `name` is not
/// present in `scalar_names`.
pub fn seed_selection(cli: &Cli, scalar_names: &[String]) -> Result<Selection> {
    let scalar_index = match cli.scalar.as_deref() {
        None => 0,
        Some(name) => match scalar_names.iter().position(|n| n == name) {
            Some(i) => i,
            None => {
                if scalar_names.is_empty() {
                    bail!("--scalar={name:?} requested, but the PLY carries no per-vertex scalars",);
                } else {
                    bail!(
                        "--scalar={name:?} not found in PLY. Available scalars: {}",
                        scalar_names.join(", "),
                    );
                }
            }
        },
    };
    Ok(Selection {
        scalar_index,
        colormap_override: cli.colormap.into(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(args: &[&str]) -> Result<Cli> {
        let mut full = vec!["cf-view"];
        full.extend_from_slice(args);
        Ok(Cli::try_parse_from(full)?)
    }

    #[test]
    fn cli_default_flags() -> Result<()> {
        let cli = parse(&["/some/file.ply"])?;
        assert_eq!(cli.path, PathBuf::from("/some/file.ply"));
        assert_eq!(cli.scalar, None);
        assert_eq!(cli.colormap, ColormapArg::Auto);
        assert_eq!(cli.up, UpAxisArg::PlusZ);
        Ok(())
    }

    #[test]
    fn cli_parses_scalar_flag() -> Result<()> {
        let cli = parse(&["/x.ply", "--scalar=signed_distance"])?;
        assert_eq!(cli.scalar.as_deref(), Some("signed_distance"));
        Ok(())
    }

    #[test]
    fn cli_parses_each_colormap_value() -> Result<()> {
        for (s, expected) in [
            ("auto", ColormapArg::Auto),
            ("divergent", ColormapArg::Divergent),
            ("sequential", ColormapArg::Sequential),
            ("categorical", ColormapArg::Categorical),
        ] {
            let cli = parse(&["/x.ply", "--colormap", s])?;
            assert_eq!(cli.colormap, expected, "colormap={s}");
        }
        Ok(())
    }

    #[test]
    fn cli_parses_each_up_axis_value() -> Result<()> {
        for (s, expected) in [
            ("+X", UpAxisArg::PlusX),
            ("+Y", UpAxisArg::PlusY),
            ("+Z", UpAxisArg::PlusZ),
        ] {
            let cli = parse(&["/x.ply", "--up", s])?;
            assert_eq!(cli.up, expected, "up={s}");
        }
        Ok(())
    }

    #[test]
    fn cli_missing_path_errors() {
        let result = Cli::try_parse_from(["cf-view"]);
        assert!(result.is_err(), "missing path should error");
    }

    #[test]
    fn colormap_arg_maps_to_override() {
        assert_eq!(
            ColormapOverride::from(ColormapArg::Auto),
            ColormapOverride::Auto
        );
        assert_eq!(
            ColormapOverride::from(ColormapArg::Divergent),
            ColormapOverride::Divergent,
        );
        assert_eq!(
            ColormapOverride::from(ColormapArg::Sequential),
            ColormapOverride::Sequential,
        );
        assert_eq!(
            ColormapOverride::from(ColormapArg::Categorical),
            ColormapOverride::Categorical,
        );
    }

    #[test]
    fn up_axis_arg_maps_to_resource() {
        assert_eq!(UpAxis::from(UpAxisArg::PlusX), UpAxis::PlusX);
        assert_eq!(UpAxis::from(UpAxisArg::PlusY), UpAxis::PlusY);
        assert_eq!(UpAxis::from(UpAxisArg::PlusZ), UpAxis::PlusZ);
    }

    fn cli_with(scalar: Option<&str>, colormap: ColormapArg) -> Cli {
        Cli {
            path: PathBuf::from("/x.ply"),
            scalar: scalar.map(str::to_string),
            colormap,
            up: UpAxisArg::PlusZ,
        }
    }

    #[test]
    fn seed_selection_default_picks_first_scalar() -> Result<()> {
        let names = vec!["alpha".to_string(), "beta".to_string()];
        let s = seed_selection(&cli_with(None, ColormapArg::Auto), &names)?;
        assert_eq!(s.scalar_index, 0);
        assert_eq!(s.colormap_override, ColormapOverride::Auto);
        Ok(())
    }

    #[test]
    fn seed_selection_resolves_named_scalar() -> Result<()> {
        let names = vec!["alpha".to_string(), "beta".to_string(), "gamma".to_string()];
        let s = seed_selection(&cli_with(Some("beta"), ColormapArg::Sequential), &names)?;
        assert_eq!(s.scalar_index, 1);
        assert_eq!(s.colormap_override, ColormapOverride::Sequential);
        Ok(())
    }

    #[test]
    fn seed_selection_errors_on_unknown_scalar() {
        let names = vec!["phi".to_string(), "zeta".to_string()];
        let result = seed_selection(&cli_with(Some("nope"), ColormapArg::Auto), &names);
        assert!(result.is_err(), "expected unknown-scalar error");
        let msg = result.err().map(|e| e.to_string()).unwrap_or_default();
        assert!(
            msg.contains("nope"),
            "error message should name the bad scalar: {msg}"
        );
        assert!(
            msg.contains("phi") && msg.contains("zeta"),
            "error message should list available scalars: {msg}",
        );
    }

    #[test]
    fn seed_selection_errors_when_no_scalars_present() {
        let names: Vec<String> = Vec::new();
        let result = seed_selection(&cli_with(Some("anything"), ColormapArg::Auto), &names);
        assert!(result.is_err(), "expected no-scalars error");
        let msg = result.err().map(|e| e.to_string()).unwrap_or_default();
        assert!(
            msg.contains("no per-vertex scalars"),
            "error message should explain the empty case: {msg}",
        );
    }

    #[test]
    fn seed_selection_passes_colormap_through() -> Result<()> {
        let names = vec!["alpha".to_string()];
        for (arg, expected) in [
            (ColormapArg::Auto, ColormapOverride::Auto),
            (ColormapArg::Divergent, ColormapOverride::Divergent),
            (ColormapArg::Sequential, ColormapOverride::Sequential),
            (ColormapArg::Categorical, ColormapOverride::Categorical),
        ] {
            let s = seed_selection(&cli_with(None, arg), &names)?;
            assert_eq!(s.colormap_override, expected);
        }
        Ok(())
    }
}
