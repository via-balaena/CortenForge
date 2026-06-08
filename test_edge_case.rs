use cf_msk_lib::anthro::{AnthroSource, Sex};
use cf_msk_lib::Template;
use nalgebra::Vector3;
use cf_msk_lib::{Body, Model, ir::{TransformAxis, TransformFn}};

fn chain_template() -> Template {
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

fn main() {
    let t = chain_template();
    
    // Test boundary: 0.0 should pass assertion
    println!("Testing 0.0...");
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let _params = AnthroSource::new(Sex::Male, 0.0).params(&t);
    })) {
        Ok(_) => println!("0.0 passed assertion"),
        Err(_) => println!("0.0 panicked"),
    }
    
    // Test boundary: 1.0 should pass assertion
    println!("Testing 1.0...");
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let _params = AnthroSource::new(Sex::Male, 1.0).params(&t);
    })) {
        Ok(_) => println!("1.0 passed assertion"),
        Err(_) => println!("1.0 panicked"),
    }
    
    // Test just inside: 0.001 
    println!("Testing 0.001...");
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let _params = AnthroSource::new(Sex::Male, 0.001).params(&t);
    })) {
        Ok(_) => println!("0.001 passed"),
        Err(_) => println!("0.001 panicked"),
    }
    
    // Test just inside: 0.999
    println!("Testing 0.999...");
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let _params = AnthroSource::new(Sex::Male, 0.999).params(&t);
    })) {
        Ok(_) => println!("0.999 passed"),
        Err(_) => println!("0.999 panicked"),
    }
}
