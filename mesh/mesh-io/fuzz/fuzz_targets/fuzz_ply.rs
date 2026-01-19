//! Fuzz target for PLY file parsing.
//!
//! Run with: cargo +nightly fuzz run fuzz_ply

#![no_main]

use libfuzzer_sys::fuzz_target;
use std::io::Write;
use tempfile::NamedTempFile;

fuzz_target!(|data: &[u8]| {
    // Write fuzz data to a temporary file with .ply extension
    let mut file = match NamedTempFile::with_suffix(".ply") {
        Ok(f) => f,
        Err(_) => return,
    };

    if file.write_all(data).is_err() {
        return;
    }

    // Try to load the PLY file - should not panic regardless of input
    let _ = mesh_io::load_mesh(file.path());
});
