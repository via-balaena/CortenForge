//! `cf-cast-cli` binary — scan → cast bridge entry point.
//!
//! `cf-cast-cli <cast.toml> [--output-dir <dir>]` reads the cast TOML
//! and any cf-scan-prep outputs it references, derives a v2.1
//! [`cf_cast::CastSpec`] + [`cf_cast::Ribbon`], writes mold pieces +
//! per-layer plugs + procedure markdown to disk, then prints a
//! summary.

use std::path::PathBuf;
use std::process::ExitCode;

use anyhow::Result;
use clap::Parser;

#[derive(Debug, Parser)]
#[command(
    name = "cf-cast-cli",
    about = "Scan → cast bridge: turn a cf-scan-prep cleaned STL + .prep.toml + cast.toml into v2.1 mold piece STLs + plugs + procedure.md.",
    long_about = None,
)]
struct Cli {
    /// Path to the cast TOML describing the multi-layer cast.
    cast_toml: PathBuf,

    /// Override the output directory for the generated mold pieces,
    /// plugs, and procedure markdown. When omitted, falls back to
    /// the `cast.output_dir` field in the cast TOML (default `"out"`
    /// relative to the cast TOML's directory).
    #[arg(long)]
    output_dir: Option<PathBuf>,
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e:#}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<()> {
    let cli = Cli::parse();
    let report = cf_cast_cli::run(&cli.cast_toml, cli.output_dir.as_deref())?;

    println!("cf-cast-cli — v2.1 scan-driven cast artifacts:");
    println!();
    println!("Output directory: {}", report.out_dir.display());
    println!();
    for layer in &report.v2.layers {
        let mass_g = layer.pour_volume.pour_mass_kg * 1000.0;
        println!(
            "  layer {} ({}): {:.2} g",
            layer.layer_index, layer.material_display_name, mass_g,
        );
        for piece in &layer.pieces {
            println!("    {:?}  →  {}", piece.piece_side, piece.path.display());
        }
        println!("    plug      →  {}", layer.plug.path.display());
    }
    println!("  procedure  →  {}", report.procedure_path.display());
    println!();
    println!(
        "Total silicone mass: {:.2} g across {} layers.",
        report.total_mass_g, report.layer_count,
    );
    println!();
    println!(
        "Centerline arc length: {:.1} mm; max tangent rotation: {:.1}°.",
        report.arc_length_mm, report.max_tangent_rotation_deg,
    );
    Ok(())
}
