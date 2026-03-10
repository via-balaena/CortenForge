//! Smoke test: verify all conformance models parse and forward() without error.

use sim_mjcf::load_model;

const MODELS: &[(&str, &str)] = &[
    (
        "pendulum",
        include_str!("../assets/golden/conformance/models/pendulum.xml"),
    ),
    (
        "double_pendulum",
        include_str!("../assets/golden/conformance/models/double_pendulum.xml"),
    ),
    (
        "contact_scenario",
        include_str!("../assets/golden/conformance/models/contact_scenario.xml"),
    ),
    (
        "actuated_system",
        include_str!("../assets/golden/conformance/models/actuated_system.xml"),
    ),
    (
        "tendon_model",
        include_str!("../assets/golden/conformance/models/tendon_model.xml"),
    ),
    (
        "sensor_model",
        include_str!("../assets/golden/conformance/models/sensor_model.xml"),
    ),
    (
        "equality_model",
        include_str!("../assets/golden/conformance/models/equality_model.xml"),
    ),
    (
        "composite_model",
        include_str!("../assets/golden/conformance/models/composite_model.xml"),
    ),
];

#[test]
fn conformance_models_parse_and_forward() {
    for (name, xml) in MODELS {
        let model = load_model(xml).unwrap_or_else(|e| panic!("{name}: parse error: {e}"));
        let mut data = model.make_data();
        data.forward(&model)
            .unwrap_or_else(|e| panic!("{name}: forward error: {e:?}"));
        println!(
            "  OK  {name}: nq={} nv={} nu={}",
            model.nq, model.nv, model.nu
        );
    }
}
