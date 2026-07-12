//! Sim-to-real ingestion for the **EMPS** actuator benchmark (step 2 of the rung:
//! fetch + parse; the identification loop is added in step 3).
//!
//! The Electro-Mechanical Positioning System (EMPS) is a prismatic drive whose
//! dominant nonlinearity is dry friction — the same physics [`FrictionSystemId`]
//! already identifies, now on a *forced* system (the drive force is a measured
//! input). This example loads the measured trajectory the identification consumes.
//!
//! Data: A. Janot, M. Gautier, M. Brunot, *Data Set and Reference Models of an
//! Electro-Mechanical Positioning System*, 2019 Workshop on Nonlinear System
//! Identification Benchmarks. It is **referenced, not redistributed**: the archive
//! is fetched at runtime (no data bytes committed). `DATA_EMPS.mat` (MATLAB v5)
//! holds `t` (s), `vir` (drive input), `qm` (measured position, m), and `gtau`
//! (input→force gain); the generalized force on the mass is `gtau · vir`.
//!
//! ```text
//! cargo run -p cf-codesign --example emps_sim_to_real --release
//! ```
//! Requires `curl` + `unzip` and one-time network access (then cached in the temp
//! dir). Offline: download the URL below, `unzip -p` `DATA_EMPS.mat` to the cache path.
#![allow(clippy::expect_used, clippy::print_stdout)]

use std::path::PathBuf;
use std::process::Command;

/// The nonlinearbenchmark EMPS distribution (a zip containing `DATA_EMPS.mat`).
/// Referenced, never vendored.
const DATA_URL: &str =
    "https://drive.google.com/uc?export=download&id=1zwoXYa9-3f8NQ0ohzmjpF7UxbNgRTHkS";

/// One EMPS trajectory: time (s), drive input `vir`, measured position `qm` (m),
/// and the input→force gain `gtau` (generalized force = `gtau · vir`).
struct EmpsData {
    t: Vec<f64>,
    vir: Vec<f64>,
    qm: Vec<f64>,
    gtau: f64,
}

/// Fetch `DATA_EMPS.mat` to a temp-dir cache (reused on later runs), extracting it
/// from the referenced archive. Returns `None` with a guidance message if
/// `curl`/`unzip`/network is unavailable — an example must not panic on that.
fn fetch_emps() -> Option<PathBuf> {
    let cache = std::env::temp_dir().join("cf_emps_DATA_EMPS.mat");
    if cache.exists() {
        return Some(cache);
    }
    let zip = std::env::temp_dir().join("cf_emps.zip");
    if !zip.exists() {
        println!("Fetching EMPS dataset → {}", zip.display());
        // Download to a temp name and rename on success so an interrupted transfer
        // never leaves a truncated file a later run would trust.
        let part = zip.with_extension("part");
        let ok = Command::new("curl")
            .args(["-sSL", "-o", part.to_str().expect("utf-8 path"), DATA_URL])
            .status()
            .map(|s| s.success())
            .unwrap_or(false);
        if !ok || std::fs::rename(&part, &zip).is_err() {
            println!("  could not fetch (need `curl` + network); skipping.");
            return None;
        }
    }
    // Extract just the one member to the cache path.
    let out = Command::new("unzip")
        .args(["-p", zip.to_str().expect("utf-8 path"), "DATA_EMPS.mat"])
        .output()
        .ok()?;
    if !out.status.success() || out.stdout.is_empty() {
        println!("  could not extract DATA_EMPS.mat (need `unzip`); skipping.");
        return None;
    }
    std::fs::write(&cache, &out.stdout).ok()?;
    Some(cache)
}

/// Parse the required arrays out of the MATLAB v5 `.mat`.
fn load_emps(path: &PathBuf) -> EmpsData {
    let mat =
        matfile::MatFile::parse(std::fs::File::open(path).expect("open .mat")).expect("parse .mat");
    let col = |name: &str| -> Vec<f64> {
        match mat
            .find_by_name(name)
            .unwrap_or_else(|| panic!("missing `{name}`"))
            .data()
        {
            matfile::NumericData::Double { real, .. } => real.clone(),
            other => panic!("`{name}` is not a double array: {other:?}"),
        }
    };
    let gtau = col("gtau")[0];
    EmpsData {
        t: col("t"),
        vir: col("vir"),
        qm: col("qm"),
        gtau,
    }
}

fn range(x: &[f64]) -> (f64, f64) {
    x.iter()
        .fold((f64::MAX, f64::MIN), |(a, b), &v| (a.min(v), b.max(v)))
}

fn main() {
    let Some(path) = fetch_emps() else { return };
    let d = load_emps(&path);
    let n = d.t.len();
    let dt = (d.t[n - 1] - d.t[0]) / (n - 1) as f64;
    let (vmin, vmax) = range(&d.vir);
    let (qmin, qmax) = range(&d.qm);

    println!("\n=== EMPS ingestion (Janot et al. 2019) ===");
    println!(
        "{n} samples @ {:.0} Hz over {:.2} s",
        1.0 / dt,
        d.t[n - 1] - d.t[0]
    );
    println!(
        "  input   vir  ∈ [{vmin:.3}, {vmax:.3}]   gain gtau = {:.3}",
        d.gtau
    );
    println!(
        "  force   gtau·vir ∈ [{:.2}, {:.2}]  (generalized drive on the mass)",
        d.gtau * vmin,
        d.gtau * vmax
    );
    println!("  output  qm   ∈ [{qmin:.4}, {qmax:.4}] m  (measured position, the fit target)");
    println!("\nIngestion OK — ready for the friction-ID loop (step 3).");
}
