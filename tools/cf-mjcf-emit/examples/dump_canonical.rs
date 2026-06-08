//! Print the canonical (no-scan) knee MJCF to stdout.
//!
//! Doubles as the generator for the frozen reference snapshot:
//! `cargo run -p cf-mjcf-emit --example dump_canonical > tools/cf-mjcf-emit/tests/assets/knee_ref.xml`
//!
//! It parses the vendored gait2392 into the [`Model`](cf_msk_lib::Model) IR and
//! runs the builder-first path (`CanonicalSource → realize → emit`) — the same
//! call `build_canonical` makes.

use cf_mjcf_emit::build_canonical;
use cf_osim::parse_leg_chain;

fn main() {
    let path = format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392/gait2392.osim",
        env!("CARGO_MANIFEST_DIR")
    );
    let model = parse_leg_chain(&std::fs::read_to_string(path).expect("read gait2392.osim"));
    print!("{}", build_canonical(&model).mjcf);
}
