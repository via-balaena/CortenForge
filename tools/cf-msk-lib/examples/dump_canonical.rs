//! Print the canonical (no-scan) knee MJCF to stdout.
//!
//! Doubles as the generator for the frozen reference snapshot:
//! `cargo run -p cf-msk-lib --example dump_canonical > tools/cf-msk-lib/tests/assets/knee_ref.xml`
//!
//! It parses the vendored gait2392 template and runs the builder-first path
//! (`CanonicalSource → realize → emit`) — the same call `build_canonical` makes.

use cf_msk_lib::build_canonical;
use cf_osim::osim::parse_knee_subgraph;

fn main() {
    let path = format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392/gait2392.osim",
        env!("CARGO_MANIFEST_DIR")
    );
    let template = parse_knee_subgraph(&std::fs::read_to_string(path).expect("read gait2392.osim"));
    print!("{}", build_canonical(&template).mjcf);
}
