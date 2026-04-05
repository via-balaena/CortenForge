//! Integration tests for mesh-cylinder and mesh-ellipsoid GJK/EPA dispatch.
//!
//! Verifies that cylinder and ellipsoid geoms collide correctly with mesh
//! geoms in full MJCF-based simulation (settling under gravity).
//!
//! Run with:
//! ```sh
//! cargo test -p sim-conformance-tests --test integration mesh_cylinder_ellipsoid -- --nocapture
//! ```

use sim_core::ConstraintType;
use sim_mjcf::load_model;

/// Inline MJCF vertex/face strings for a box mesh with given half-sizes.
/// Centered at origin; 8 vertices, 12 triangles.
fn box_mesh_mjcf_strings(hx: f64, hy: f64, hz: f64) -> (String, String) {
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

/// Measure steady-state contact force over the last `n_steps - settle_steps` steps.
/// Returns (average_contact_force, final_body_z_position, final_body_z_velocity).
fn settle_and_measure(
    model: &sim_core::Model,
    body_id: usize,
    n_steps: usize,
    settle_steps: usize,
) -> (f64, f64, f64) {
    let mut data = model.make_data();

    let mut total_contact_force = 0.0_f64;
    let measure_steps = n_steps - settle_steps;
    let mut final_z = 0.0_f64;
    let mut final_vz = 0.0_f64;

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
            // Body velocity: first 3 components of cvel are angular, next 3 are linear
            final_vz = data.cvel[body_id][5];
        }
    }

    #[allow(clippy::cast_precision_loss)]
    let avg_force = total_contact_force / measure_steps as f64;
    (avg_force, final_z, final_vz)
}

// ============================================================================
// T10: Cylinder on mesh-plane settling
// ============================================================================

/// T10: Upright cylinder on mesh ground plane settles at correct height.
///
/// Cylinder: half_length=0.2, radius=0.1, density=1000.
/// Mesh ground: thin box slab at z=0 (top face).
/// Expected rest height: center at z = half_length = 0.2.
/// Contact force ≈ weight.
///
/// With cylinder MULTICCD face enumeration (8 cap rim points), the flat cap
/// produces multiple contacts providing rotational stability.
#[test]
fn mesh_cylinder_settling() {
    let half_length: f64 = 0.2;
    let radius: f64 = 0.1;
    let density: f64 = 1000.0;
    let gravity: f64 = 9.81;
    let n_steps: usize = 3000;
    let settle_steps: usize = 2500;

    // Ground slab: thick (2×2×2) so cylinder can't tunnel through.
    // Positioned at z=-1.0 so top face is at z=0.
    let (gnd_verts, gnd_faces) = box_mesh_mjcf_strings(1.0, 1.0, 1.0);

    // Start cylinder at rest height: center at z=half_length so bottom cap
    // is at z=0 (mesh surface). MULTICCD provides flat-cap stability.
    let mjcf = format!(
        r#"<mujoco>
  <option gravity="0 0 -{gravity}" timestep="0.001">
    <flag multiccd="enable"/>
  </option>
  <asset>
    <mesh name="ground" vertex="{gnd_verts}" face="{gnd_faces}"/>
  </asset>
  <worldbody>
    <geom name="floor" type="mesh" mesh="ground" pos="0 0 -1.0"/>
    <body name="cylinder" pos="0 0 {half_length}">
      <joint type="free"/>
      <geom type="cylinder" size="{radius} {half_length}" density="{density}"/>
    </body>
  </worldbody>
</mujoco>"#
    );

    let model = load_model(&mjcf).expect("model should load");

    let body_mass = model.body_mass[1];
    let weight = body_mass * gravity;

    // body_id=1 (worldbody=0, cylinder=1)
    let (avg_force, final_z, final_vz) = settle_and_measure(&model, 1, n_steps, settle_steps);

    eprintln!(
        "T10: z={final_z:.4}, vz={final_vz:.6}, force={avg_force:.4}, weight={weight:.4}, ratio={:.4}",
        avg_force / weight
    );

    // Rest height: center at z ≈ half_length = 0.2 (within 3mm)
    assert!(
        (final_z - half_length).abs() < 0.003,
        "cylinder center should rest at z≈{half_length}, got z={final_z:.4}"
    );

    // Settled: velocity near zero
    assert!(
        final_vz.abs() < 0.01,
        "should be settled (vz < 0.01), got vz={final_vz:.6}"
    );

    // Contact force ≈ weight (within 5%)
    let force_ratio = avg_force / weight;
    assert!(
        (0.95..=1.05).contains(&force_ratio),
        "force/weight ratio should be ~1.0, got {force_ratio:.4}"
    );
}

// ============================================================================
// T11: Ellipsoid on mesh-plane settling
// ============================================================================

/// T11: Oblate ellipsoid on mesh ground settles at correct height.
///
/// Ellipsoid: radii [0.3, 0.3, 0.05], density=1000.
/// Expected rest height: center at z = rz = 0.05, NOT max_r = 0.3.
#[test]
fn mesh_ellipsoid_settling() {
    let rx: f64 = 0.3;
    let ry: f64 = 0.3;
    let rz: f64 = 0.05;
    let density: f64 = 1000.0;
    let gravity: f64 = 9.81;
    let n_steps: usize = 3000;
    let settle_steps: usize = 2500;

    let (gnd_verts, gnd_faces) = box_mesh_mjcf_strings(1.0, 1.0, 1.0);

    let mjcf = format!(
        r#"<mujoco>
  <option gravity="0 0 -{gravity}" timestep="0.001"/>
  <asset>
    <mesh name="ground" vertex="{gnd_verts}" face="{gnd_faces}"/>
  </asset>
  <worldbody>
    <geom name="floor" type="mesh" mesh="ground" pos="0 0 -1.0"/>
    <body name="ellipsoid" pos="0 0 0.15">
      <joint type="free"/>
      <geom type="ellipsoid" size="{rx} {ry} {rz}" density="{density}"/>
    </body>
  </worldbody>
</mujoco>"#
    );

    let model = load_model(&mjcf).expect("model should load");
    let body_mass = model.body_mass[1];
    let weight = body_mass * gravity;

    let (avg_force, final_z, final_vz) = settle_and_measure(&model, 1, n_steps, settle_steps);

    eprintln!(
        "T11: z={final_z:.4}, vz={final_vz:.6}, force={avg_force:.4}, weight={weight:.4}, ratio={:.4}",
        avg_force / weight
    );

    // Rest height: center at z ≈ rz = 0.05 (within 3mm)
    // Old sphere approx would give z ≈ 0.3 — completely wrong
    assert!(
        (final_z - rz).abs() < 0.003,
        "ellipsoid center should rest at z≈{rz}, got z={final_z:.4} (sphere approx would give ~0.3)"
    );

    // Settled
    assert!(
        final_vz.abs() < 0.01,
        "should be settled (vz < 0.01), got vz={final_vz:.6}"
    );

    // Contact force ≈ weight (within 5%)
    let force_ratio = avg_force / weight;
    assert!(
        (0.95..=1.05).contains(&force_ratio),
        "force/weight ratio should be ~1.0, got {force_ratio:.4}"
    );
}

// ============================================================================
// T12: Mesh-Cylinder MULTICCD
// ============================================================================

/// T12: MULTICCD produces contacts for cylinder flat cap on mesh.
///
/// Enable MULTICCD, drop cylinder onto mesh ground. The flat cap face
/// should produce >= 1 contact.
#[test]
fn mesh_cylinder_multiccd() {
    let half_length: f64 = 0.2;
    let radius: f64 = 0.1;
    let density: f64 = 1000.0;
    let gravity: f64 = 9.81;

    let (gnd_verts, gnd_faces) = box_mesh_mjcf_strings(1.0, 1.0, 1.0);

    let mjcf = format!(
        r#"<mujoco>
  <option gravity="0 0 -{gravity}" timestep="0.001">
    <flag multiccd="enable"/>
  </option>
  <asset>
    <mesh name="ground" vertex="{gnd_verts}" face="{gnd_faces}"/>
  </asset>
  <worldbody>
    <geom name="floor" type="mesh" mesh="ground" pos="0 0 -1.0"/>
    <body name="cylinder" pos="0 0 0.3">
      <joint type="free"/>
      <geom type="cylinder" size="{radius} {half_length}" density="{density}"/>
    </body>
  </worldbody>
</mujoco>"#
    );

    let model = load_model(&mjcf).expect("model should load");

    // Simulate until settled
    let mut data = model.make_data();
    for _ in 0..1000 {
        data.step(&model).expect("step failed");
    }

    // At steady state, should have at least one contact
    assert!(
        !data.contacts.is_empty(),
        "MULTICCD mesh-cylinder should produce at least 1 contact"
    );

    // All contacts should have reasonable normals (roughly +Z or -Z)
    for c in &data.contacts {
        assert!(
            c.normal.z.abs() > 0.9,
            "contact normal should be along Z, got {:?}",
            c.normal
        );
    }
}
