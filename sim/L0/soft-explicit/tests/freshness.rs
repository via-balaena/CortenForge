//! The committed `src/shared.wgsl` must be what `sim-wgsl-gen` generates from
//! `src/shared/`, and every shared file must be both compiled and translated.

#![allow(clippy::unwrap_used, clippy::panic)]

/// The shared sources, in translation order, relative to this crate.
const SOURCES: [&str; 8] = [
    "src/shared/linalg.rs",
    "src/shared/material.rs",
    "src/shared/element.rs",
    "src/shared/anp.rs",
    "src/shared/integrate.rs",
    "src/shared/pose.rs",
    "src/shared/sdf.rs",
    "src/shared/contact.rs",
];

/// This crate's path from the workspace root, which is where the
/// regenerate command runs and what the WGSL header records.
const CRATE: &str = "sim/L0/soft-explicit";

fn read(relative: &str) -> String {
    std::fs::read_to_string(format!("{}/{relative}", env!("CARGO_MANIFEST_DIR"))).unwrap()
}

#[test]
fn the_committed_wgsl_is_what_the_shared_sources_generate() {
    let paths: Vec<String> = SOURCES.iter().map(|s| format!("{CRATE}/{s}")).collect();
    let texts: Vec<String> = SOURCES.iter().map(|s| read(s)).collect();
    let sources: Vec<sim_wgsl_gen::Source<'_>> = paths
        .iter()
        .zip(&texts)
        .map(|(path, text)| sim_wgsl_gen::Source { path, text })
        .collect();
    let generated = sim_wgsl_gen::translate(&sources).unwrap_or_else(|e| panic!("{e}"));
    if let Some(difference) =
        sim_wgsl_gen::first_difference(sim_soft_explicit::SHARED_WGSL, &generated)
    {
        let inputs: Vec<&str> = paths.iter().map(String::as_str).collect();
        panic!(
            "{CRATE}/src/shared.wgsl is stale ({difference}).\nRegenerate it from the workspace root:\n    {}",
            sim_wgsl_gen::regenerate_command(&format!("{CRATE}/src/shared.wgsl"), &inputs)
        );
    }
}

/// A shared file that is compiled but not translated (or the reverse) would
/// let the CPU and GPU drift apart unnoticed.
#[test]
fn every_shared_file_is_compiled_at_both_precisions_and_translated() {
    let lib = read("src/lib.rs");
    for module in ["pub mod f32 {", "pub mod f64 {"] {
        let start = lib.find(module).unwrap();
        let body = &lib[start..start + lib[start..].find("\n}").unwrap()];
        let included: Vec<&str> = body
            .lines()
            .filter_map(|line| line.trim().strip_prefix("include!(\""))
            .map(|rest| rest.trim_end_matches("\");"))
            .collect();
        let expected: Vec<String> = SOURCES
            .iter()
            .map(|s| s.trim_start_matches("src/").to_string())
            .collect();
        assert_eq!(
            included, expected,
            "{module} includes differ from the translated list"
        );
    }
    let on_disk = std::fs::read_dir(format!("{}/src/shared", env!("CARGO_MANIFEST_DIR")))
        .unwrap()
        .count();
    assert_eq!(
        on_disk,
        SOURCES.len(),
        "a file in src/shared/ is neither compiled nor translated"
    );
}
