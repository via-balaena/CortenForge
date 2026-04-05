//! Stress test — headless validation of all 7 mesh collision pairs + edge cases.
//!
//! 21 checks covering: mesh-plane (3), mesh-sphere (2), mesh-box (2),
//! mesh-capsule (2), mesh-cylinder (3), mesh-ellipsoid (3), mesh-mesh (3),
//! edge cases (3).
//!
//! Run: `cargo run -p example-mesh-collision-stress-test --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::similar_names,
    clippy::too_many_lines,
    clippy::suboptimal_flops,
    clippy::let_underscore_must_use
)]

use sim_core::ConstraintType;
use sim_mjcf::load_model;

// ── Helpers ────────────────────────────────────────────────────────────────

fn check(name: &str, pass: bool, detail: &str) -> bool {
    let tag = if pass { "PASS" } else { "FAIL" };
    println!("  [{tag}] {name}: {detail}");
    pass
}

/// Inline MJCF vertex/face strings for a box mesh with given half-sizes.
/// Centered at origin; 8 vertices, 12 triangles.
fn box_mesh_verts_faces(hx: f64, hy: f64, hz: f64) -> (String, String) {
    let vertices = [
        [-hx, -hy, -hz],
        [hx, -hy, -hz],
        [hx, hy, -hz],
        [-hx, hy, -hz],
        [-hx, -hy, hz],
        [hx, -hy, hz],
        [hx, hy, hz],
        [-hx, hy, hz],
    ];
    #[rustfmt::skip]
    let faces: [[usize; 3]; 12] = [
        [0,1,2], [0,2,3], // -Z
        [4,6,5], [4,7,6], // +Z
        [0,4,5], [0,5,1], // -Y
        [2,6,7], [2,7,3], // +Y
        [0,3,7], [0,7,4], // -X
        [1,5,6], [1,6,2], // +X
    ];

    let vert_str: String = vertices
        .iter()
        .map(|v| format!("{:.6} {:.6} {:.6}", v[0], v[1], v[2]))
        .collect::<Vec<_>>()
        .join("  ");

    let face_str: String = faces
        .iter()
        .map(|f| format!("{} {} {}", f[0], f[1], f[2]))
        .collect::<Vec<_>>()
        .join("  ");

    (vert_str, face_str)
}

/// Simulate for `n_steps`, measure steady-state contact force over last
/// `n_steps - settle_steps` steps.
/// Returns (avg_contact_force, final_body_z, final_body_vz, final_ncon).
fn settle_and_measure(
    model: &sim_core::Model,
    body_id: usize,
    n_steps: usize,
    settle_steps: usize,
) -> (f64, f64, f64, usize) {
    let mut data = model.make_data();
    let mut total_contact_force = 0.0_f64;
    let measure_steps = n_steps - settle_steps;
    let mut final_z = 0.0_f64;
    let mut final_vz = 0.0_f64;
    let mut final_ncon = 0_usize;

    for step in 0..n_steps {
        data.step(model).expect("step failed");

        if step >= settle_steps {
            for (i, ct) in data.efc_type.iter().enumerate() {
                if matches!(
                    ct,
                    ConstraintType::ContactFrictionless
                        | ConstraintType::ContactPyramidal
                        | ConstraintType::ContactElliptic
                ) {
                    total_contact_force += data.efc_force[i].abs();
                }
            }
        }

        if step == n_steps - 1 {
            final_z = data.xpos[body_id].z;
            final_vz = data.cvel[body_id][5];
            final_ncon = data.contacts.len();
        }
    }

    let avg_force = total_contact_force / measure_steps as f64;
    (avg_force, final_z, final_vz, final_ncon)
}

// ============================================================================
// Group A: Mesh-Plane (checks 1-3)
// ============================================================================

fn check_mesh_plane() -> (u32, u32) {
    println!("\n--- A: Mesh-Plane ---");

    // Centered tetrahedron (same as visual example)
    // Rest height = sqrt(6)/12 ≈ 0.204124
    let rest_height: f64 = 0.204_124;
    let gravity: f64 = 9.81;

    let mjcf = format!(
        r#"<mujoco>
  <option gravity="0 0 -{gravity}" timestep="0.001" integrator="RK4"/>
  <asset>
    <mesh name="tetra"
          vertex="-0.5 -0.28868 -0.20412
                   0.5 -0.28868 -0.20412
                   0.0  0.57735 -0.20412
                   0.0  0.0      0.61237"
          face="0 2 1  0 1 3  1 2 3  0 3 2"/>
  </asset>
  <worldbody>
    <geom type="plane" size="2 2 0.01"/>
    <body name="tetra" pos="0 0 0.5">
      <joint type="free"/>
      <geom type="mesh" mesh="tetra" density="1000"/>
    </body>
  </worldbody>
</mujoco>"#
    );

    let model = load_model(&mjcf).expect("model should load");
    let weight = model.body_mass[1] * gravity;
    let (avg_force, z, _vz, ncon) = settle_and_measure(&model, 1, 3000, 2500);
    let force_ratio = avg_force / weight;

    let mut pass = 0u32;
    let mut total = 0u32;

    total += 1;
    if check(
        "1. Rest height",
        (z - rest_height).abs() < 0.003,
        &format!(
            "z={z:.4}, expect≈{rest_height:.3}, err={:.4}",
            (z - rest_height).abs()
        ),
    ) {
        pass += 1;
    }

    total += 1;
    if check(
        "2. Contact force/weight",
        (0.95..=1.05).contains(&force_ratio),
        &format!("ratio={force_ratio:.4}"),
    ) {
        pass += 1;
    }

    total += 1;
    if check("3. Contacts exist", ncon >= 1, &format!("ncon={ncon}")) {
        pass += 1;
    }

    (pass, total)
}

// ============================================================================
// Group B: Mesh-Sphere (checks 4-5)
// ============================================================================

fn check_mesh_sphere() -> (u32, u32) {
    println!("\n--- B: Mesh-Sphere ---");

    let radius: f64 = 0.1;
    let gravity: f64 = 9.81;
    let (gnd_v, gnd_f) = box_mesh_verts_faces(1.0, 1.0, 1.0);

    let mjcf = format!(
        r#"<mujoco>
  <option gravity="0 0 -{gravity}" timestep="0.001" integrator="RK4"/>
  <asset>
    <mesh name="ground" vertex="{gnd_v}" face="{gnd_f}"/>
  </asset>
  <worldbody>
    <geom type="mesh" mesh="ground" pos="0 0 -1"/>
    <body name="ball" pos="0 0 0.3">
      <joint type="free"/>
      <geom type="sphere" size="{radius}" density="1000"/>
    </body>
  </worldbody>
</mujoco>"#
    );

    let model = load_model(&mjcf).expect("model should load");
    let weight = model.body_mass[1] * gravity;
    let (avg_force, z, _vz, _ncon) = settle_and_measure(&model, 1, 3000, 2500);
    let force_ratio = avg_force / weight;

    let mut pass = 0u32;
    let mut total = 0u32;

    total += 1;
    if check(
        "4. Rest height",
        (z - radius).abs() < 0.003,
        &format!("z={z:.4}, expect≈{radius}"),
    ) {
        pass += 1;
    }

    total += 1;
    if check(
        "5. Contact force/weight",
        (0.95..=1.05).contains(&force_ratio),
        &format!("ratio={force_ratio:.4}"),
    ) {
        pass += 1;
    }

    (pass, total)
}

// ============================================================================
// Group C: Mesh-Box (checks 6-7)
// Now uses GJK/EPA on mesh convex hull (same path as cylinder/ellipsoid).
// MULTICCD required for flat box face on flat mesh face stability.
// ============================================================================

fn check_mesh_box() -> (u32, u32) {
    println!("\n--- C: Mesh-Box (GJK/EPA + MULTICCD) ---");

    let half_z: f64 = 0.05;
    let gravity: f64 = 9.81;
    let (gnd_v, gnd_f) = box_mesh_verts_faces(1.0, 1.0, 1.0);

    let mjcf = format!(
        r#"<mujoco>
  <option gravity="0 0 -{gravity}" timestep="0.001" integrator="RK4">
    <flag multiccd="enable"/>
  </option>
  <asset>
    <mesh name="ground" vertex="{gnd_v}" face="{gnd_f}"/>
  </asset>
  <worldbody>
    <geom type="mesh" mesh="ground" pos="0 0 -1"/>
    <body name="box" pos="0 0 0.3">
      <joint type="free"/>
      <geom type="box" size="0.15 0.1 {half_z}" density="1000"/>
    </body>
  </worldbody>
</mujoco>"#
    );

    let model = load_model(&mjcf).expect("model should load");
    let weight = model.body_mass[1] * gravity;
    let (avg_force, z, _vz, _ncon) = settle_and_measure(&model, 1, 3000, 2500);
    let force_ratio = avg_force / weight;

    let mut pass = 0u32;
    let mut total = 0u32;

    total += 1;
    if check(
        "6. Rest height",
        (z - half_z).abs() < 0.003,
        &format!("z={z:.4}, expect≈{half_z}"),
    ) {
        pass += 1;
    }

    total += 1;
    if check(
        "7. Contact force/weight",
        (0.95..=1.05).contains(&force_ratio),
        &format!("ratio={force_ratio:.4}"),
    ) {
        pass += 1;
    }

    (pass, total)
}

// ============================================================================
// Group D: Mesh-Capsule (checks 8-9)
// ============================================================================

fn check_mesh_capsule() -> (u32, u32) {
    println!("\n--- D: Mesh-Capsule ---");

    // Sideways capsule: axis along Y, rest height = radius.
    // Barrel-on-mesh contact is ~4.5mm less precise than endpoint contact
    // (curved surface vs flat triangle approximation), so we use 5mm tolerance.
    let radius: f64 = 0.1;
    let gravity: f64 = 9.81;
    let (gnd_v, gnd_f) = box_mesh_verts_faces(1.0, 1.0, 1.0);

    let mjcf = format!(
        r#"<mujoco>
  <option gravity="0 0 -{gravity}" timestep="0.001" integrator="RK4"/>
  <asset>
    <mesh name="ground" vertex="{gnd_v}" face="{gnd_f}"/>
  </asset>
  <worldbody>
    <geom type="mesh" mesh="ground" pos="0 0 -1"/>
    <body name="capsule" pos="0 0 0.2" quat="0.707 0.707 0 0">
      <joint type="free"/>
      <geom type="capsule" size="{radius} 0.2" density="1000"/>
    </body>
  </worldbody>
</mujoco>"#
    );

    let model = load_model(&mjcf).expect("model should load");
    let weight = model.body_mass[1] * gravity;
    let (avg_force, z, _vz, _ncon) = settle_and_measure(&model, 1, 3000, 2500);
    let force_ratio = avg_force / weight;

    let mut pass = 0u32;
    let mut total = 0u32;

    total += 1;
    if check(
        "8. Rest height (5mm — barrel-on-mesh)",
        (z - radius).abs() < 0.005,
        &format!("z={z:.4}, expect≈{radius}, err={:.4}", (z - radius).abs()),
    ) {
        pass += 1;
    }

    total += 1;
    if check(
        "9. Contact force/weight",
        (0.95..=1.05).contains(&force_ratio),
        &format!("ratio={force_ratio:.4}"),
    ) {
        pass += 1;
    }

    (pass, total)
}

// ============================================================================
// Group E: Mesh-Cylinder (checks 10-12)
// ============================================================================

fn check_mesh_cylinder() -> (u32, u32) {
    println!("\n--- E: Mesh-Cylinder ---");

    let half_length: f64 = 0.2;
    let radius: f64 = 0.1;
    let gravity: f64 = 9.81;
    let (gnd_v, gnd_f) = box_mesh_verts_faces(1.0, 1.0, 1.0);

    let mjcf = format!(
        r#"<mujoco>
  <option gravity="0 0 -{gravity}" timestep="0.001" integrator="RK4">
    <flag multiccd="enable"/>
  </option>
  <asset>
    <mesh name="ground" vertex="{gnd_v}" face="{gnd_f}"/>
  </asset>
  <worldbody>
    <geom type="mesh" mesh="ground" pos="0 0 -1"/>
    <body name="cylinder" pos="0 0 {half_length}">
      <joint type="free"/>
      <geom type="cylinder" size="{radius} {half_length}" density="1000"/>
    </body>
  </worldbody>
</mujoco>"#
    );

    let model = load_model(&mjcf).expect("model should load");
    let weight = model.body_mass[1] * gravity;
    let (avg_force, z, _vz, ncon) = settle_and_measure(&model, 1, 3000, 2500);
    let force_ratio = avg_force / weight;

    let mut pass = 0u32;
    let mut total = 0u32;

    total += 1;
    if check(
        "10. Rest height (GJK/EPA)",
        (z - half_length).abs() < 0.003,
        &format!("z={z:.4}, expect≈{half_length}"),
    ) {
        pass += 1;
    }

    total += 1;
    if check(
        "11. Contact force/weight",
        (0.95..=1.05).contains(&force_ratio),
        &format!("ratio={force_ratio:.4}"),
    ) {
        pass += 1;
    }

    total += 1;
    if check("12. MULTICCD contacts", ncon >= 2, &format!("ncon={ncon}")) {
        pass += 1;
    }

    (pass, total)
}

// ============================================================================
// Group F: Mesh-Ellipsoid (checks 13-15)
// ============================================================================

fn check_mesh_ellipsoid() -> (u32, u32) {
    println!("\n--- F: Mesh-Ellipsoid ---");

    let rz: f64 = 0.05;
    let gravity: f64 = 9.81;
    let (gnd_v, gnd_f) = box_mesh_verts_faces(1.0, 1.0, 1.0);

    let mjcf = format!(
        r#"<mujoco>
  <option gravity="0 0 -{gravity}" timestep="0.001" integrator="RK4"/>
  <asset>
    <mesh name="ground" vertex="{gnd_v}" face="{gnd_f}"/>
  </asset>
  <worldbody>
    <geom type="mesh" mesh="ground" pos="0 0 -1"/>
    <body name="ellipsoid" pos="0 0 0.15">
      <joint type="free"/>
      <geom type="ellipsoid" size="0.3 0.3 {rz}" density="1000"/>
    </body>
  </worldbody>
</mujoco>"#
    );

    let model = load_model(&mjcf).expect("model should load");
    let weight = model.body_mass[1] * gravity;
    let (avg_force, z, _vz, _ncon) = settle_and_measure(&model, 1, 3000, 2500);
    let force_ratio = avg_force / weight;

    let mut pass = 0u32;
    let mut total = 0u32;

    total += 1;
    if check(
        "13. Rest height (GJK/EPA)",
        (z - rz).abs() < 0.003,
        &format!("z={z:.4}, expect≈{rz}"),
    ) {
        pass += 1;
    }

    total += 1;
    if check(
        "14. Rejects sphere approx",
        z < 0.1,
        &format!("z={z:.4} (sphere approx would give ~0.3)"),
    ) {
        pass += 1;
    }

    total += 1;
    if check(
        "15. Contact force/weight",
        (0.95..=1.05).contains(&force_ratio),
        &format!("ratio={force_ratio:.4}"),
    ) {
        pass += 1;
    }

    (pass, total)
}

// ============================================================================
// Group G: Mesh-Mesh (checks 16-18)
// MULTICCD required: hull-hull GJK/EPA returns a single contact point, which
// is insufficient for face-face stacking. MULTICCD generates multiple points
// along flat faces, just like it does for cylinder caps.
// ============================================================================

fn check_mesh_mesh() -> (u32, u32) {
    println!("\n--- G: Mesh-Mesh (MULTICCD) ---");

    let gravity: f64 = 9.81;

    // Platform: box mesh 1x1x0.1 at z=0.05 (top face at z=0.1)
    let (plat_v, plat_f) = box_mesh_verts_faces(0.5, 0.5, 0.05);

    // Wedge: triangular prism, 6 vertices, 8 faces.
    // Base at z=0 in local frame, apex at z=0.25.
    let wedge_v = "0 -0.15 0  0.3 -0.15 0  0.15 -0.15 0.25  0 0.15 0  0.3 0.15 0  0.15 0.15 0.25";
    let wedge_f = "0 1 2  3 5 4  0 3 4  0 4 1  1 4 5  1 5 2  2 5 3  2 3 0";

    let mjcf = format!(
        r#"<mujoco>
  <option gravity="0 0 -{gravity}" timestep="0.001" integrator="RK4">
    <flag multiccd="enable"/>
  </option>
  <asset>
    <mesh name="platform" vertex="{plat_v}" face="{plat_f}"/>
    <mesh name="wedge" vertex="{wedge_v}" face="{wedge_f}"/>
  </asset>
  <worldbody>
    <geom type="plane" size="2 2 0.01"/>
    <geom type="mesh" mesh="platform" pos="0 0 0.05"/>
    <body name="wedge" pos="0 0 0.35">
      <joint type="free"/>
      <geom type="mesh" mesh="wedge" density="1000"/>
    </body>
  </worldbody>
</mujoco>"#
    );

    let model = load_model(&mjcf).expect("model should load");
    let weight = model.body_mass[1] * gravity;
    let (avg_force, z, vz, _ncon) = settle_and_measure(&model, 1, 3000, 2500);
    let force_ratio = avg_force / weight;

    let mut pass = 0u32;
    let mut total = 0u32;

    total += 1;
    if check("16. Settled", vz.abs() < 0.01, &format!("vz={vz:.6}")) {
        pass += 1;
    }

    total += 1;
    if check(
        "17. On platform (5mm tolerance)",
        z > 0.095,
        &format!("z={z:.4} (platform top=0.1, solver compliance ~0.1mm)"),
    ) {
        pass += 1;
    }

    total += 1;
    if check(
        "18. Contact force/weight",
        (0.90..=1.10).contains(&force_ratio),
        &format!("ratio={force_ratio:.4} (10% tolerance for hull approx)"),
    ) {
        pass += 1;
    }

    (pass, total)
}

// ============================================================================
// Group H: Edge Cases (checks 19-21)
// ============================================================================

fn check_edge_cases() -> (u32, u32) {
    println!("\n--- H: Edge Cases ---");

    let gravity: f64 = 9.81;
    let mut pass = 0u32;
    let mut total = 0u32;

    // Check 19: Two meshes far apart — no contacts
    {
        let (v, f) = box_mesh_verts_faces(0.1, 0.1, 0.1);
        let mjcf = format!(
            r#"<mujoco>
  <option gravity="0 0 -{gravity}" timestep="0.001"/>
  <asset>
    <mesh name="a" vertex="{v}" face="{f}"/>
    <mesh name="b" vertex="{v}" face="{f}"/>
  </asset>
  <worldbody>
    <body name="a" pos="0 0 5">
      <joint type="free"/>
      <geom type="mesh" mesh="a" density="1000"/>
    </body>
    <body name="b" pos="0 0 -5">
      <joint type="free"/>
      <geom type="mesh" mesh="b" density="1000"/>
    </body>
  </worldbody>
</mujoco>"#
        );

        let model = load_model(&mjcf).expect("model should load");
        let mut data = model.make_data();
        // Single step — they're far apart, no gravity settling needed
        data.step(&model).expect("step failed");
        let ncon = data.contacts.len();

        total += 1;
        if check("19. Separated meshes", ncon == 0, &format!("ncon={ncon}")) {
            pass += 1;
        }
    }

    // Check 20: Single-triangle mesh vs sphere (BVH path, no hull needed)
    {
        // 3 vertices, 1 face — below the 4-point hull minimum, but BVH path works
        let tri_v = "-0.5 -0.5 0.0  0.5 -0.5 0.0  0.0 0.5 0.0";
        let tri_f = "0 1 2";

        let mjcf = format!(
            r#"<mujoco>
  <option gravity="0 0 -{gravity}" timestep="0.001" integrator="RK4"/>
  <asset>
    <mesh name="tri" vertex="{tri_v}" face="{tri_f}"/>
  </asset>
  <worldbody>
    <geom type="mesh" mesh="tri"/>
    <body name="ball" pos="0 0 0.2">
      <joint type="free"/>
      <geom type="sphere" size="0.1" density="1000"/>
    </body>
  </worldbody>
</mujoco>"#
        );

        let model = load_model(&mjcf).expect("model should load");
        let mut data = model.make_data();
        // Run a few steps to get contact
        for _ in 0..500 {
            data.step(&model).expect("step failed");
        }
        let ncon = data.contacts.len();

        total += 1;
        if check(
            "20. Single-tri mesh vs sphere",
            ncon >= 1,
            &format!("ncon={ncon} (BVH path, no hull required)"),
        ) {
            pass += 1;
        }
    }

    // Check 21: Swapped geom order — sphere before mesh in worldbody
    // Should produce same rest height as mesh-sphere (check 4)
    {
        let radius: f64 = 0.1;
        let (gnd_v, gnd_f) = box_mesh_verts_faces(1.0, 1.0, 1.0);

        // Sphere geom is on the worldbody (geom1), mesh body is second (geom2).
        // This tests the (Primitive, Mesh) dispatch path with normal negation.
        let mjcf = format!(
            r#"<mujoco>
  <option gravity="0 0 -{gravity}" timestep="0.001" integrator="RK4"/>
  <asset>
    <mesh name="ground" vertex="{gnd_v}" face="{gnd_f}"/>
  </asset>
  <worldbody>
    <body name="ball" pos="0 0 0.3">
      <joint type="free"/>
      <geom type="sphere" size="{radius}" density="1000"/>
    </body>
    <geom type="mesh" mesh="ground" pos="0 0 -1"/>
  </worldbody>
</mujoco>"#
        );

        let model = load_model(&mjcf).expect("model should load");
        let (_avg_force, z, _vz, _ncon) = settle_and_measure(&model, 1, 3000, 2500);

        total += 1;
        if check(
            "21. Swapped order (sphere,mesh)",
            (z - radius).abs() < 0.001,
            &format!("z={z:.4}, expect≈{radius} (< 1mm)"),
        ) {
            pass += 1;
        }
    }

    (pass, total)
}

// ============================================================================
// Main
// ============================================================================

fn main() {
    println!("=== CortenForge: Mesh Collision Stress Test ===");
    println!("  21 headless checks across all 7 mesh collision pairs + edge cases\n");

    let mut total_pass = 0u32;
    let mut total_checks = 0u32;

    let groups: Vec<fn() -> (u32, u32)> = vec![
        check_mesh_plane,
        check_mesh_sphere,
        check_mesh_box,
        check_mesh_capsule,
        check_mesh_cylinder,
        check_mesh_ellipsoid,
        check_mesh_mesh,
        check_edge_cases,
    ];

    for group in groups {
        let (p, t) = group();
        total_pass += p;
        total_checks += t;
    }

    println!("\n=== Summary: {total_pass}/{total_checks} PASS ===");

    if total_pass < total_checks {
        std::process::exit(1);
    }
}
