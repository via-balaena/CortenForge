//! Error Handling
//!
//! Feeds several invalid URDFs to the parser and verifies each produces the
//! expected error variant — not a panic. The crate should fail gracefully
//! with actionable error messages.
//!
//! Run: `cargo run -p example-urdf-error-handling --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::suboptimal_flops,
    clippy::too_many_lines,
    clippy::unnecessary_unwrap
)]

use sim_urdf::{UrdfError, UrdfJoint, UrdfJointType, UrdfLink, UrdfRobot};

fn check(name: &str, pass: bool, detail: &str) -> bool {
    let tag = if pass { "PASS" } else { "FAIL" };
    println!("  [{tag}] {name}: {detail}");
    pass
}

fn main() {
    println!("=== URDF Loading — Error Handling ===\n");

    let mut passed = 0u32;
    let mut total = 0u32;

    // --- Check 1: Missing <robot> element ---
    let result = sim_urdf::parse_urdf_str("<link name='test'/>");
    let is_err = result.is_err();
    if check(
        "Missing <robot> → error",
        is_err,
        &if is_err {
            format!("{}", result.unwrap_err())
        } else {
            "BUG: should have failed".into()
        },
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 2: Unknown joint type ---
    let result = sim_urdf::parse_urdf_str(
        r#"
        <robot name="test">
            <link name="a"/>
            <link name="b"/>
            <joint name="j" type="invalid_type">
                <parent link="a"/>
                <child link="b"/>
            </joint>
        </robot>
    "#,
    );
    let is_unknown = matches!(&result, Err(UrdfError::UnknownJointType(_)));
    if check(
        "Unknown joint type → UnknownJointType",
        is_unknown,
        &if result.is_err() {
            format!("{}", result.unwrap_err())
        } else {
            "BUG: should have failed".into()
        },
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 3: Undefined link reference ---
    let robot = UrdfRobot::new("test")
        .with_link(UrdfLink::new("base"))
        .with_joint(UrdfJoint::new(
            "j1",
            UrdfJointType::Fixed,
            "base",
            "nonexistent",
        ));
    let result = sim_urdf::validate(&robot);
    let is_undef = matches!(&result, Err(UrdfError::UndefinedLink { .. }));
    if check(
        "Undefined link → UndefinedLink",
        is_undef,
        &if result.is_err() {
            format!("{}", result.unwrap_err())
        } else {
            "BUG: should have failed".into()
        },
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 4: Duplicate link names ---
    let robot = UrdfRobot::new("test")
        .with_link(UrdfLink::new("same_name"))
        .with_link(UrdfLink::new("same_name"));
    let result = sim_urdf::validate(&robot);
    let is_dup = matches!(&result, Err(UrdfError::DuplicateLink(_)));
    if check(
        "Duplicate link → DuplicateLink",
        is_dup,
        &if result.is_err() {
            format!("{}", result.unwrap_err())
        } else {
            "BUG: should have failed".into()
        },
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 5: Malformed XML ---
    let result = sim_urdf::parse_urdf_str("<not valid xml");
    let is_xml_err = result.is_err();
    if check(
        "Malformed XML → error (no panic)",
        is_xml_err,
        &if is_xml_err {
            format!("{}", result.unwrap_err())
        } else {
            "BUG: should have failed".into()
        },
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 6: Empty string ---
    let result = sim_urdf::parse_urdf_str("");
    let is_empty_err = result.is_err();
    if check(
        "Empty string → error (no panic)",
        is_empty_err,
        &if is_empty_err {
            format!("{}", result.unwrap_err())
        } else {
            "BUG: should have failed".into()
        },
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 7: Error messages are human-readable ---
    let err = UrdfError::undefined_link("arm_link", "shoulder_joint");
    let msg = err.to_string();
    let has_link = msg.contains("arm_link");
    let has_joint = msg.contains("shoulder_joint");
    if check(
        "Error message includes context",
        has_link && has_joint,
        &format!("msg=\"{msg}\""),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Summary ---
    println!("\n============================================================");
    println!("  TOTAL: {passed}/{total} checks passed");
    if passed == total {
        println!("  ALL PASS");
    } else {
        println!("  {} FAILED", total - passed);
        std::process::exit(1);
    }
}
