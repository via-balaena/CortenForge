//! Dial a few anthropometric bodies from the generator — no scan — and emit each.
//!
//!     cargo run -p cf-mjcf-emit --example dial_body
//!
//! Shows the A3 builder-first generator end to end: a sex/percentile `AnthroSource`
//! → `realize` → MJCF that the importer loads, with the dialed segment lengths.

use cf_mjcf_emit::build;
use cf_msk_lib::anthro::{AnthroSource, Sex};
use cf_msk_lib::{ParamSource, realize};
use cf_osim::parse_leg_chain;
use sim_mjcf::load_model;

fn main() {
    let path = format!(
        "{}/../../sim/L0/tests/assets/opensim_gait2392/gait2392.osim",
        env!("CARGO_MANIFEST_DIR")
    );
    let t = parse_leg_chain(&std::fs::read_to_string(path).expect("read gait2392.osim"));

    let bodies = [
        ("male p50 (reference)", AnthroSource::new(Sex::Male, 0.50)),
        ("male p95", AnthroSource::new(Sex::Male, 0.95)),
        ("female p05", AnthroSource::new(Sex::Female, 0.05)),
        (
            "male p50 stocky (girth p95)",
            AnthroSource::new(Sex::Male, 0.50).with_girth_percentile(0.95),
        ),
    ];

    println!("{:<28} {:>8} {:>8}  loads", "body", "femur_mm", "tibia_mm");
    for (label, src) in bodies {
        let model = realize(&t, &src.params(&t));
        let femur = model.segment_axial_length("femur_r", "tibia_r") * 1000.0;
        let tibia = model.segment_axial_length("tibia_r", "talus_r") * 1000.0;
        let ok = load_model(&build(&t, &src).mjcf).is_ok();
        println!("{label:<28} {femur:>8.1} {tibia:>8.1}  {ok}");
    }
}
