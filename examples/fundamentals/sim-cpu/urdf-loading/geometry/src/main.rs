//! Primitive Geometry Shapes
//!
//! One arm with box (upper arm), cylinder (forearm), sphere (hand). Verifies
//! all three primitives appear in the compiled model with correct sizes after
//! the URDF → MJCF size convention conversion:
//!
//! - Box: URDF full-extents → MJCF half-extents
//! - Cylinder: URDF (radius, length) → MJCF (radius, half-length)
//! - Sphere: radius passes through unchanged
//!
//! Run: `cargo run -p example-urdf-geometry --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::suboptimal_flops
)]

use sim_core::GeomType;

/// Arm with three geometry types: box base, sphere link, cylinder link.
const GEOMETRY_URDF: &str = r#"<?xml version="1.0"?>
<robot name="geometry_test">
    <link name="base">
        <inertial>
            <mass value="5.0"/>
            <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.5"/>
        </inertial>
        <collision>
            <geometry>
                <box size="0.2 0.4 0.6"/>
            </geometry>
        </collision>
    </link>
    <link name="link1">
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
        </inertial>
        <collision>
            <geometry>
                <sphere radius="0.15"/>
            </geometry>
        </collision>
    </link>
    <link name="link2">
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
        </inertial>
        <collision>
            <geometry>
                <cylinder radius="0.08" length="0.3"/>
            </geometry>
        </collision>
    </link>
    <joint name="j1" type="revolute">
        <parent link="base"/>
        <child link="link1"/>
        <origin xyz="0 0 0.3"/>
        <axis xyz="0 1 0"/>
        <limit lower="-1.57" upper="1.57"/>
    </joint>
    <joint name="j2" type="revolute">
        <parent link="link1"/>
        <child link="link2"/>
        <origin xyz="0 0 0.3"/>
        <axis xyz="0 1 0"/>
        <limit lower="-1.57" upper="1.57"/>
    </joint>
</robot>
"#;

fn check(name: &str, pass: bool, detail: &str) -> bool {
    let tag = if pass { "PASS" } else { "FAIL" };
    println!("  [{tag}] {name}: {detail}");
    pass
}

fn main() {
    println!("=== URDF Loading — Primitive Geometry Shapes ===\n");

    let mut passed = 0u32;
    let mut total = 0u32;

    let model = sim_urdf::load_urdf_model(GEOMETRY_URDF).expect("URDF should parse");

    // --- Check 1: Three geoms exist ---
    let three_geoms = model.ngeom == 3;
    if check(
        "Three geoms in model",
        three_geoms,
        &format!("ngeom={}", model.ngeom),
    ) {
        passed += 1;
    }
    total += 1;

    // --- Check 2: Box half-extents ---
    // URDF box size="0.2 0.4 0.6" → MJCF half-extents 0.1, 0.2, 0.3
    let mut found_box = false;
    for i in 0..model.ngeom {
        if model.geom_type[i] == GeomType::Box {
            let s = model.geom_size[i];
            let ok =
                (s.x - 0.1).abs() < 0.001 && (s.y - 0.2).abs() < 0.001 && (s.z - 0.3).abs() < 0.001;
            if check(
                "Box half-extents (0.1, 0.2, 0.3)",
                ok,
                &format!("size=({:.3}, {:.3}, {:.3})", s.x, s.y, s.z),
            ) {
                passed += 1;
            }
            total += 1;
            found_box = true;
            break;
        }
    }
    if !found_box {
        check("Box half-extents", false, "no box geom found");
        total += 1;
    }

    // --- Check 3: Sphere radius ---
    let mut found_sphere = false;
    for i in 0..model.ngeom {
        if model.geom_type[i] == GeomType::Sphere {
            let r = model.geom_size[i].x;
            let ok = (r - 0.15).abs() < 0.001;
            if check("Sphere radius 0.15", ok, &format!("r={r:.3}")) {
                passed += 1;
            }
            total += 1;
            found_sphere = true;
            break;
        }
    }
    if !found_sphere {
        check("Sphere radius", false, "no sphere geom found");
        total += 1;
    }

    // --- Check 4: Cylinder size ---
    // URDF radius=0.08 length=0.3 → MJCF size="0.08 0.15" (radius, half_length)
    let mut found_cyl = false;
    for i in 0..model.ngeom {
        if model.geom_type[i] == GeomType::Cylinder {
            let s = model.geom_size[i];
            let r_ok = (s.x - 0.08).abs() < 0.001;
            let hl_ok = (s.y - 0.15).abs() < 0.001;
            if check(
                "Cylinder (r=0.08, hl=0.15)",
                r_ok && hl_ok,
                &format!("size=({:.3}, {:.3})", s.x, s.y),
            ) {
                passed += 1;
            }
            total += 1;
            found_cyl = true;
            break;
        }
    }
    if !found_cyl {
        check("Cylinder size", false, "no cylinder geom found");
        total += 1;
    }

    // --- Check 5: All three types present ---
    let types: Vec<GeomType> = (0..model.ngeom).map(|i| model.geom_type[i]).collect();
    let has_all = types.contains(&GeomType::Box)
        && types.contains(&GeomType::Sphere)
        && types.contains(&GeomType::Cylinder);
    if check(
        "All 3 primitive types present",
        has_all,
        &format!("types={types:?}"),
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
