//! Fuzz target for 3MF file parsing.
//!
//! Run with: cargo +nightly fuzz run fuzz_3mf

#![no_main]

use libfuzzer_sys::fuzz_target;
use std::io::Write;
use tempfile::NamedTempFile;

fuzz_target!(|data: &[u8]| {
    // Write fuzz data to a temporary file with .3mf extension
    let mut file = match NamedTempFile::with_suffix(".3mf") {
        Ok(f) => f,
        Err(_) => return,
    };

    if file.write_all(data).is_err() {
        return;
    }

    // Try to load the 3MF file - should not panic regardless of input
    let _ = mesh_io::load_mesh(file.path());
});
