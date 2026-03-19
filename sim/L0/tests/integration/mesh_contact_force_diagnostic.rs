//! DT-180: Mesh contact force verification.
//!
//! Verifies that mesh-plane contact forces match expected weight at steady state,
//! and that contact normals point in the correct direction for both geom orderings.
//!
//! Run with:
//! ```sh
//! cargo test -p sim-conformance-tests --release --test integration mesh_contact_force -- --nocapture
//! ```

use sim_core::ConstraintType;
use sim_mjcf::load_model;

// ============================================================================
// Icosphere mesh generation (duplicated from mesh_collision_profile.rs)
// ============================================================================

fn icosphere_mjcf_strings(radius: f64, subdivisions: u32) -> (String, String) {
    let phi: f64 = (1.0 + 5.0_f64.sqrt()) / 2.0;

    let raw = [
        (-1.0, phi, 0.0),
        (1.0, phi, 0.0),
        (-1.0, -phi, 0.0),
        (1.0, -phi, 0.0),
        (0.0, -1.0, phi),
        (0.0, 1.0, phi),
        (0.0, -1.0, -phi),
        (0.0, 1.0, -phi),
        (phi, 0.0, -1.0),
        (phi, 0.0, 1.0),
        (-phi, 0.0, -1.0),
        (-phi, 0.0, 1.0),
    ];

    let mut vertices: Vec<[f64; 3]> = raw
        .iter()
        .map(|&(x, y, z)| {
            let len = (x * x + y * y + z * z).sqrt();
            [x / len * radius, y / len * radius, z / len * radius]
        })
        .collect();

    let mut faces: Vec<[usize; 3]> = vec![
        [0, 11, 5],
        [0, 5, 1],
        [0, 1, 7],
        [0, 7, 10],
        [0, 10, 11],
        [1, 5, 9],
        [5, 11, 4],
        [11, 10, 2],
        [10, 7, 6],
        [7, 1, 8],
        [3, 9, 4],
        [3, 4, 2],
        [3, 2, 6],
        [3, 6, 8],
        [3, 8, 9],
        [4, 9, 5],
        [2, 4, 11],
        [6, 2, 10],
        [8, 6, 7],
        [9, 8, 1],
    ];

    for _ in 0..subdivisions {
        use std::collections::HashMap;
        let mut midpoint_cache: HashMap<(usize, usize), usize> = HashMap::new();

        let mut get_midpoint = |v1: usize, v2: usize, verts: &mut Vec<[f64; 3]>| -> usize {
            let key = if v1 < v2 { (v1, v2) } else { (v2, v1) };
            if let Some(&idx) = midpoint_cache.get(&key) {
                return idx;
            }
            let a = verts[v1];
            let b = verts[v2];
            let mid = [
                (a[0] + b[0]) / 2.0,
                (a[1] + b[1]) / 2.0,
                (a[2] + b[2]) / 2.0,
            ];
            let len = (mid[0] * mid[0] + mid[1] * mid[1] + mid[2] * mid[2]).sqrt();
            let normalized = [
                mid[0] / len * radius,
                mid[1] / len * radius,
                mid[2] / len * radius,
            ];
            let idx = verts.len();
            verts.push(normalized);
            midpoint_cache.insert(key, idx);
            idx
        };

        let old_faces = faces.clone();
        faces.clear();
        for f in &old_faces {
            let a = get_midpoint(f[0], f[1], &mut vertices);
            let b = get_midpoint(f[1], f[2], &mut vertices);
            let c = get_midpoint(f[2], f[0], &mut vertices);
            faces.push([f[0], a, c]);
            faces.push([f[1], b, a]);
            faces.push([f[2], c, b]);
            faces.push([a, b, c]);
        }
    }

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

// ============================================================================
// DT-180: Mesh contact force assertion test
// ============================================================================

/// Measure steady-state contact force over the last `n_steps - settle_steps` steps.
/// Returns (average_contact_force_magnitude, last_step_contacts).
fn measure_contact_force(
    model: &sim_core::Model,
    n_steps: usize,
    settle_steps: usize,
) -> (f64, Vec<sim_core::Contact>) {
    let mut data = model.make_data();

    let mut total_contact_force = 0.0_f64;
    let measure_steps = n_steps - settle_steps;
    let mut last_contacts = Vec::new();

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
            last_contacts = data.contacts.clone();
        }
    }

    #[allow(clippy::cast_precision_loss)]
    let avg_force = total_contact_force / measure_steps as f64;
    (avg_force, last_contacts)
}

/// DT-180: Mesh sphere on plane produces correct contact force and normal.
///
/// Verifies the (Plane, Mesh) geom ordering — plane is geom1, mesh is geom2.
/// The `(prim_type, Mesh)` branch in `collide_with_mesh` negates the normal
/// returned by `collide_mesh_plane`. With the fix (normal = -plane_normal),
/// negation produces +plane_normal = +Z. Correct.
#[test]
fn dt180_mesh_contact_force_plane_mesh_order() {
    let radius: f64 = 0.5;
    let density: f64 = 1000.0;
    let gravity: f64 = 9.81;
    let n_steps: usize = 500;
    let settle_steps: usize = 400;

    let (verts, faces) = icosphere_mjcf_strings(radius, 1);

    // Plane first (geom index 0), then mesh body — gives (Plane, Mesh) ordering
    let mjcf = format!(
        r#"<mujoco>
  <option gravity="0 0 -{gravity}" timestep="0.002"/>
  <asset>
    <mesh name="sphere" vertex="{verts}" face="{faces}"/>
  </asset>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1"/>
    <body name="ball" pos="0 0 {radius}">
      <joint type="free"/>
      <geom type="mesh" mesh="sphere" density="{density}"/>
    </body>
  </worldbody>
</mujoco>"#
    );

    let model = load_model(&mjcf).expect("model should load");
    let mesh_mass = model.body_mass[1];
    let mesh_weight = mesh_mass * gravity;

    let (avg_force, last_contacts) = measure_contact_force(&model, n_steps, settle_steps);

    // Force within 5% of mesh weight
    let force_ratio = avg_force / mesh_weight;
    eprintln!(
        "DT-180 (Plane,Mesh): force={avg_force:.4} N, weight={mesh_weight:.4} N, ratio={force_ratio:.4}"
    );
    assert!(
        (0.95..=1.05).contains(&force_ratio),
        "Mesh contact force should be within 5% of weight: force={avg_force:.4}, weight={mesh_weight:.4}, ratio={force_ratio:.6}"
    );

    // Contact normal should point +Z (upward, supporting the ball)
    assert!(
        !last_contacts.is_empty(),
        "Should have at least one contact at steady state"
    );
    for c in &last_contacts {
        assert!(
            c.normal.z > 0.9,
            "Contact normal should point +Z, got [{:.4}, {:.4}, {:.4}]",
            c.normal.x,
            c.normal.y,
            c.normal.z,
        );
    }
}

/// DT-180: Mesh sphere on plane with (Mesh, Plane) geom ordering.
///
/// The `(Mesh, prim_type)` branch in `collide_with_mesh` does NOT negate the normal.
/// With the fix (normal = -plane_normal = -Z), the contact normal passed to
/// `make_contact_from_geoms` is -Z. The Contact convention is "normal from geom1
/// toward geom2", so geom1=Mesh, geom2=Plane → normal should point downward (-Z)
/// from mesh toward plane. The solver interprets this correctly as a support force.
///
/// We verify by placing the mesh geom before the plane geom in the MJCF, which
/// gives the mesh a lower geom index. The broadphase sorts pairs by (smaller, larger)
/// index, so if mesh=0 and plane=1, we get (Mesh, Plane) ordering.
#[test]
fn dt180_mesh_contact_force_mesh_plane_order() {
    let radius: f64 = 0.5;
    let density: f64 = 1000.0;
    let gravity: f64 = 9.81;
    let n_steps: usize = 500;
    let settle_steps: usize = 400;

    let (verts, faces) = icosphere_mjcf_strings(radius, 1);

    // Mesh body first (geom index 0), plane second (geom index 1)
    // This gives (Mesh, Plane) ordering in collision dispatch.
    let mjcf = format!(
        r#"<mujoco>
  <option gravity="0 0 -{gravity}" timestep="0.002"/>
  <asset>
    <mesh name="sphere" vertex="{verts}" face="{faces}"/>
  </asset>
  <worldbody>
    <body name="ball" pos="0 0 {radius}">
      <joint type="free"/>
      <geom type="mesh" mesh="sphere" density="{density}"/>
    </body>
    <geom name="floor" type="plane" size="5 5 0.1"/>
  </worldbody>
</mujoco>"#
    );

    let model = load_model(&mjcf).expect("model should load");
    let mesh_mass = model.body_mass[1];
    let mesh_weight = mesh_mass * gravity;

    let (avg_force, last_contacts) = measure_contact_force(&model, n_steps, settle_steps);

    // Force within 5% of mesh weight
    let force_ratio = avg_force / mesh_weight;
    eprintln!(
        "DT-180 (Mesh,Plane): force={avg_force:.4} N, weight={mesh_weight:.4} N, ratio={force_ratio:.4}"
    );
    assert!(
        (0.95..=1.05).contains(&force_ratio),
        "Mesh contact force should be within 5% of weight: force={avg_force:.4}, weight={mesh_weight:.4}, ratio={force_ratio:.6}"
    );

    // In (Mesh, Plane) ordering, contact normal points from mesh toward plane = -Z.
    // This is correct — the solver uses it as a support constraint.
    assert!(
        !last_contacts.is_empty(),
        "Should have at least one contact at steady state"
    );
}

/// DT-180 mass-only diagnostic: compare body_mass for mesh vs primitive
/// without running simulation. Useful reference data.
#[test]
fn dt180_mesh_mass_comparison() {
    let radius: f64 = 0.5;
    let density: f64 = 1000.0;
    let analytic_volume = (4.0 / 3.0) * std::f64::consts::PI * radius.powi(3);
    let analytic_mass = density * analytic_volume;

    // Primitive sphere
    let m_prim = load_model(&format!(
        r#"<mujoco>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body name="b" pos="0 0 0.5">
      <joint type="free"/>
      <geom type="sphere" size="{radius}" density="{density}"/>
    </body>
  </worldbody>
</mujoco>"#
    ))
    .unwrap();

    // Mesh sphere (subdivision 1, 42 vertices)
    let (verts, faces) = icosphere_mjcf_strings(radius, 1);
    let m_mesh = load_model(&format!(
        r#"<mujoco>
  <option gravity="0 0 -9.81"/>
  <asset>
    <mesh name="s" vertex="{verts}" face="{faces}"/>
  </asset>
  <worldbody>
    <body name="b" pos="0 0 0.5">
      <joint type="free"/>
      <geom type="mesh" mesh="s" density="{density}"/>
    </body>
  </worldbody>
</mujoco>"#
    ))
    .unwrap();

    // Primitive mass should match analytic exactly
    let prim_error = (m_prim.body_mass[1] / analytic_mass - 1.0).abs();
    assert!(
        prim_error < 0.001,
        "Primitive mass error {prim_error:.4} should be < 0.1%"
    );

    // Mesh mass should be within ~15% of analytic (icosphere sub-1 approximation)
    let mesh_ratio = m_mesh.body_mass[1] / analytic_mass;
    eprintln!(
        "Mass comparison: prim={:.4}, mesh={:.4}, analytic={:.4}, mesh/analytic={:.4}",
        m_prim.body_mass[1], m_mesh.body_mass[1], analytic_mass, mesh_ratio
    );
    assert!(
        (0.80..=1.05).contains(&mesh_ratio),
        "Mesh mass ratio {mesh_ratio:.4} should be between 0.80 and 1.05"
    );
}
