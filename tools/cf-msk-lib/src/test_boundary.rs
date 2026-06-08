#[cfg(test)]
mod test_boundary_mismatch {
    use cf_msk_lib::anthro::{AnthroSource, Sex};
    use cf_msk_lib::ir::{TransformAxis, TransformFn};
    use cf_msk_lib::{Body, Model};
    use nalgebra::Vector3;

    fn chain_template() -> cf_msk_lib::Template {
        Model {
            bodies: vec![
                Body {
                    name: "pelvis".into(),
                    parent: None,
                    location_in_parent: Vector3::zeros(),
                    joint: vec![],
                },
                Body {
                    name: "femur_r".into(),
                    parent: Some(0),
                    location_in_parent: Vector3::new(0.0, -0.05, 0.0),
                    joint: vec![],
                },
                Body {
                    name: "tibia_r".into(),
                    parent: Some(1),
                    location_in_parent: Vector3::zeros(),
                    joint: vec![TransformAxis {
                        rotation: false,
                        axis: Vector3::y(),
                        coordinate: String::new(),
                        function: TransformFn::Constant(-0.45),
                    }],
                },
                Body {
                    name: "talus_r".into(),
                    parent: Some(2),
                    location_in_parent: Vector3::new(0.0, -0.40, 0.0),
                    joint: vec![],
                },
            ],
            coordinates: vec![],
            muscles: vec![],
        }
    }

    #[test]
    #[should_panic(expected = "probit domain is (0,1)")]
    fn boundary_zero_causes_probit_panic() {
        let _ = AnthroSource::new(Sex::Male, 0.0).params(&chain_template());
    }

    #[test]
    #[should_panic(expected = "probit domain is (0,1)")]
    fn boundary_one_causes_probit_panic() {
        let _ = AnthroSource::new(Sex::Male, 1.0).params(&chain_template());
    }
}
