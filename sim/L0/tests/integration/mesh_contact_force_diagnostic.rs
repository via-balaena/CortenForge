//! DT-180: Mesh contact force magnitude diagnostic.
//!
//! Compares mesh vs primitive sphere contact forces to identify the root cause
//! of the ~10^16× force magnitude bug. Primary suspect: mesh inertia/mass
//! computation producing wrong body_mass → wrong diagApprox → blown-up forces.
//!
//! Run with:
//! ```sh
//! cargo test -p sim-conformance-tests --release --test integration mesh_contact_force_diagnostic -- --nocapture
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
// DT-180 Diagnostic: Mesh vs Primitive Sphere Contact Forces
// ============================================================================

/// DT-180 diagnostic: compare mass, inertia, contact depth, and contact forces
/// between a primitive sphere and an equivalent mesh sphere on a ground plane.
///
/// Both spheres: radius=0.5, density=1000, free joint, gravity=-9.81.
/// Expected steady-state contact force ≈ weight = mass × 9.81.
#[test]
fn dt180_mesh_vs_primitive_contact_force() {
    let radius: f64 = 0.5;
    let density: f64 = 1000.0;
    let gravity: f64 = 9.81;
    let n_steps: usize = 500;
    let settle_steps: usize = 400; // Measure over last 100 steps

    // Analytic sphere properties
    let analytic_volume = (4.0 / 3.0) * std::f64::consts::PI * radius.powi(3);
    let analytic_mass = density * analytic_volume;
    let analytic_weight = analytic_mass * gravity;
    // Sphere inertia: I = (2/5) m r²
    let analytic_inertia = 0.4 * analytic_mass * radius * radius;

    eprintln!("\n=== DT-180: Mesh vs Primitive Contact Force Diagnostic ===\n");
    eprintln!("Analytic sphere (r={radius}, ρ={density}):");
    eprintln!("  volume  = {analytic_volume:.6}");
    eprintln!("  mass    = {analytic_mass:.4} kg");
    eprintln!("  weight  = {analytic_weight:.4} N");
    eprintln!("  inertia = {analytic_inertia:.6} (each diagonal)");

    // ---- Scene 1: Primitive sphere ----
    let mjcf_prim = format!(
        r#"<mujoco>
  <option gravity="0 0 -{gravity}" timestep="0.002"/>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1"/>
    <body name="ball" pos="0 0 {radius}">
      <joint type="free"/>
      <geom type="sphere" size="{radius}" density="{density}"/>
    </body>
  </worldbody>
</mujoco>"#
    );

    let m_prim = load_model(&mjcf_prim).expect("primitive model should load");

    eprintln!("\n--- Primitive sphere ---");
    eprintln!("  body_mass[1]    = {:.6}", m_prim.body_mass[1]);
    eprintln!("  body_inertia[1] = {:?}", m_prim.body_inertia[1]);

    let (prim_force, prim_contacts) = measure_contact_force(&m_prim, n_steps, settle_steps);
    eprintln!("  Steady-state contact force = {prim_force:.4} N");
    eprintln!("  Expected weight            = {analytic_weight:.4} N");
    eprintln!(
        "  Force/weight ratio         = {:.6}",
        prim_force / analytic_weight
    );
    for c in &prim_contacts {
        eprintln!(
            "  Contact: depth={:.6}, normal=[{:.3},{:.3},{:.3}], pos=[{:.3},{:.3},{:.3}]",
            c.depth, c.normal.x, c.normal.y, c.normal.z, c.pos.x, c.pos.y, c.pos.z,
        );
    }

    // ---- Scene 2: Mesh sphere (icosphere subdivision 1) ----
    let (verts, faces) = icosphere_mjcf_strings(radius, 1);

    let mjcf_mesh = format!(
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

    let m_mesh = load_model(&mjcf_mesh).expect("mesh model should load");

    eprintln!("\n--- Mesh sphere (42 vertices, subdivision 1) ---");
    eprintln!("  body_mass[1]    = {:.6e}", m_mesh.body_mass[1]);
    eprintln!(
        "  body_inertia[1] = [{:.6e}, {:.6e}, {:.6e}]",
        m_mesh.body_inertia[1].x, m_mesh.body_inertia[1].y, m_mesh.body_inertia[1].z,
    );

    let (mesh_force, mesh_contacts) = measure_contact_force(&m_mesh, n_steps, settle_steps);
    eprintln!("  Steady-state contact force = {mesh_force:.6e} N");
    eprintln!("  Expected weight            = {analytic_weight:.4} N");
    eprintln!(
        "  Force/weight ratio         = {:.6e}",
        mesh_force / analytic_weight
    );
    for c in &mesh_contacts {
        eprintln!(
            "  Contact: depth={:.6e}, normal=[{:.3},{:.3},{:.3}], pos=[{:.3},{:.3},{:.3}], dim={}, geom1={}, geom2={}",
            c.depth,
            c.normal.x,
            c.normal.y,
            c.normal.z,
            c.pos.x,
            c.pos.y,
            c.pos.z,
            c.dim,
            c.geom1,
            c.geom2,
        );
    }

    // ---- Comparison ----
    let mass_ratio = m_mesh.body_mass[1] / m_prim.body_mass[1];
    let force_ratio = mesh_force / prim_force.max(1e-30);

    eprintln!("\n--- Comparison ---");
    eprintln!("  Mass ratio  (mesh/prim) = {mass_ratio:.6e}");
    eprintln!("  Force ratio (mesh/prim) = {force_ratio:.6e}");
    eprintln!(
        "  Prim mass error  = {:.2}%",
        (m_prim.body_mass[1] / analytic_mass - 1.0).abs() * 100.0
    );
    eprintln!(
        "  Mesh mass error  = {:.2}%",
        (m_mesh.body_mass[1] / analytic_mass - 1.0).abs() * 100.0
    );

    // Diagnosis: if mass is correct but force is wrong, the bug is downstream.
    // If mass is wrong, the bug is in mesh inertia computation.
    if !(0.5..=2.0).contains(&mass_ratio) {
        eprintln!("\n  *** ROOT CAUSE: mesh body_mass is {mass_ratio:.2e}× the primitive ***");
        eprintln!("  *** Mesh inertia computation is producing wrong volume/mass ***");
    } else if !(0.1..=10.0).contains(&force_ratio) {
        eprintln!("\n  *** Mass is OK but force is {force_ratio:.2e}× off ***");
        eprintln!("  *** Bug is downstream of mass: contact params, diagApprox, or solver ***");
    } else {
        eprintln!("\n  Forces match within 10×. May not be a bug at this scale.");
    }

    // ---- Early-step contact details (before divergence) ----
    eprintln!("\n--- Early-step contact details ---");

    // Primitive: run 1 step and dump contacts
    let mut d_prim = m_prim.make_data();
    d_prim.step(&m_prim).expect("step");
    eprintln!("\nPrimitive (step 1):");
    for (i, c) in d_prim.contacts.iter().enumerate() {
        eprintln!(
            "  contact[{i}]: geom1={}, geom2={}, normal=[{:.4},{:.4},{:.4}], depth={:.6e}, dim={}, pos=[{:.4},{:.4},{:.4}]",
            c.geom1,
            c.geom2,
            c.normal.x,
            c.normal.y,
            c.normal.z,
            c.depth,
            c.dim,
            c.pos.x,
            c.pos.y,
            c.pos.z,
        );
    }
    eprintln!(
        "  ncon={}, efc_type count={}",
        d_prim.ncon,
        d_prim.efc_type.len()
    );
    for (i, ct) in d_prim.efc_type.iter().enumerate() {
        eprintln!("  efc[{i}]: type={ct:?}, force={:.6e}", d_prim.efc_force[i]);
    }

    // Mesh: run 1 step and dump contacts
    let mut d_mesh = m_mesh.make_data();
    d_mesh.step(&m_mesh).expect("step");
    eprintln!("\nMesh (step 1):");
    for (i, c) in d_mesh.contacts.iter().enumerate() {
        eprintln!(
            "  contact[{i}]: geom1={}, geom2={}, normal=[{:.4},{:.4},{:.4}], depth={:.6e}, dim={}, pos=[{:.4},{:.4},{:.4}]",
            c.geom1,
            c.geom2,
            c.normal.x,
            c.normal.y,
            c.normal.z,
            c.depth,
            c.dim,
            c.pos.x,
            c.pos.y,
            c.pos.z,
        );
    }
    eprintln!(
        "  ncon={}, efc_type count={}",
        d_mesh.ncon,
        d_mesh.efc_type.len()
    );
    for (i, ct) in d_mesh.efc_type.iter().enumerate() {
        eprintln!("  efc[{i}]: type={ct:?}, force={:.6e}", d_mesh.efc_force[i]);
    }

    // Check key steps: early, mid, and measurement window
    eprintln!("\nMesh ball step-by-step details:");
    let sample_steps = [2, 5, 10, 50, 100, 200, 300, 400, 410, 450, 490, 500];
    for step in 2..=500 {
        d_mesh.step(&m_mesh).expect("step");
        if sample_steps.contains(&step) {
            let z = d_mesh.qpos[2];
            let vz = d_mesh.qvel[2];
            let ncon = d_mesh.ncon;
            let force: f64 = d_mesh
                .efc_type
                .iter()
                .enumerate()
                .filter(|(_, ct)| {
                    matches!(
                        ct,
                        ConstraintType::ContactFrictionless
                            | ConstraintType::ContactPyramidal
                            | ConstraintType::ContactElliptic
                    )
                })
                .map(|(i, _)| d_mesh.efc_force[i].abs())
                .sum();
            eprintln!("  step {step}: z={z:.6e}, vz={vz:.6e}, ncon={ncon}, force={force:.6e}");
            for (i, c) in d_mesh.contacts.iter().enumerate() {
                eprintln!(
                    "    ct[{i}]: g1={} g2={} n=[{:.4},{:.4},{:.4}] depth={:.6e} dim={}",
                    c.geom1, c.geom2, c.normal.x, c.normal.y, c.normal.z, c.depth, c.dim,
                );
            }
            if !d_mesh.efc_type.is_empty() {
                eprintln!(
                    "    efc rows: {} (forces: [{:.3e}, ...])",
                    d_mesh.efc_type.len(),
                    d_mesh.efc_force[0],
                );
            }
        }
    }

    // ---- Root cause documentation ----
    eprintln!("\n=== DT-180 ROOT CAUSE ===");
    eprintln!();
    eprintln!("  The contact normal is INVERTED for mesh-plane collisions.");
    eprintln!("  Primitive: normal=[0,0,+1] (UP, correct)");
    eprintln!("  Mesh:      normal=[0,0,-1] (DOWN, wrong)");
    eprintln!();
    eprintln!("  Bug location: sim/L0/core/src/collision/mesh_collide.rs lines 225-228");
    eprintln!("  The (Plane, Mesh) branch negates the MeshContact normal.");
    eprintln!("  But collide_mesh_plane() returns plane_normal (+Z), not");
    eprintln!("  'from mesh outward' like other mesh collision functions.");
    eprintln!("  Negating +Z gives -Z → contact force pushes ball DOWN.");
    eprintln!();
    eprintln!("  Fix: change collide_mesh_plane to return -plane_normal");
    eprintln!("  (from mesh outward, consistent with other mesh collision fns).");
    eprintln!("  Then the existing negation in (Plane,Mesh) branch produces +Z.");
    eprintln!("  The (Mesh,Plane) branch (no negation) also gets -plane_normal = correct.");
    eprintln!();
    eprintln!("  Mass comparison (NOT the root cause):");
    eprintln!(
        "    Primitive: {:.4} kg (analytic: {:.4})",
        m_prim.body_mass[1], analytic_mass
    );
    eprintln!(
        "    Mesh:      {:.4} kg (87.3% of analytic — expected icosphere approx error)",
        m_mesh.body_mass[1]
    );
    eprintln!();
    eprintln!("=== DT-180 Diagnostic Complete ===\n");
}

/// Measure steady-state contact force over the last `n_steps - settle_steps` steps.
///
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
            // Sum contact constraint forces from efc_force
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

/// DT-180 mass-only diagnostic: compare body_mass for mesh vs primitive
/// without running simulation. This isolates the inertia computation from
/// the contact force pipeline.
#[test]
fn dt180_mesh_mass_comparison() {
    eprintln!("\n=== DT-180: Mass Comparison (no simulation) ===\n");

    let radius: f64 = 0.5;
    let density: f64 = 1000.0;
    let analytic_volume = (4.0 / 3.0) * std::f64::consts::PI * radius.powi(3);
    let analytic_mass = density * analytic_volume;

    eprintln!("Analytic: volume={analytic_volume:.6}, mass={analytic_mass:.4}\n");

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

    eprintln!("Primitive sphere:");
    eprintln!("  body_mass[1]    = {:.6}", m_prim.body_mass[1]);
    eprintln!("  body_inertia[1] = {:?}", m_prim.body_inertia[1]);

    // Mesh sphere — default inertia mode (Convex)
    let (verts, faces) = icosphere_mjcf_strings(radius, 1);
    let m_mesh_default = load_model(&format!(
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

    eprintln!("\nMesh sphere (default=convex inertia):");
    eprintln!("  body_mass[1]    = {:.6e}", m_mesh_default.body_mass[1]);
    eprintln!(
        "  body_inertia[1] = [{:.6e}, {:.6e}, {:.6e}]",
        m_mesh_default.body_inertia[1].x,
        m_mesh_default.body_inertia[1].y,
        m_mesh_default.body_inertia[1].z,
    );
    eprintln!(
        "  mass ratio (mesh/analytic) = {:.6e}",
        m_mesh_default.body_mass[1] / analytic_mass
    );

    // Mesh sphere — exact inertia mode
    let m_mesh_exact = load_model(&format!(
        r#"<mujoco>
  <option gravity="0 0 -9.81"/>
  <compiler exactmeshinertia="true"/>
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

    eprintln!("\nMesh sphere (exact inertia):");
    eprintln!("  body_mass[1]    = {:.6e}", m_mesh_exact.body_mass[1]);
    eprintln!(
        "  body_inertia[1] = [{:.6e}, {:.6e}, {:.6e}]",
        m_mesh_exact.body_inertia[1].x,
        m_mesh_exact.body_inertia[1].y,
        m_mesh_exact.body_inertia[1].z,
    );
    eprintln!(
        "  mass ratio (mesh/analytic) = {:.6e}",
        m_mesh_exact.body_mass[1] / analytic_mass
    );

    // Mesh sphere — with explicit mass (bypasses volume computation)
    let m_mesh_explicit = load_model(&format!(
        r#"<mujoco>
  <option gravity="0 0 -9.81"/>
  <asset>
    <mesh name="s" vertex="{verts}" face="{faces}"/>
  </asset>
  <worldbody>
    <body name="b" pos="0 0 0.5">
      <joint type="free"/>
      <geom type="mesh" mesh="s" mass="{analytic_mass}" density="{density}"/>
    </body>
  </worldbody>
</mujoco>"#
    ))
    .unwrap();

    eprintln!("\nMesh sphere (explicit mass={analytic_mass:.4}):");
    eprintln!("  body_mass[1]    = {:.6e}", m_mesh_explicit.body_mass[1]);
    eprintln!(
        "  body_inertia[1] = [{:.6e}, {:.6e}, {:.6e}]",
        m_mesh_explicit.body_inertia[1].x,
        m_mesh_explicit.body_inertia[1].y,
        m_mesh_explicit.body_inertia[1].z,
    );

    // Subdivision 2 (162 vertices — better sphere approximation)
    let (verts2, faces2) = icosphere_mjcf_strings(radius, 2);
    let m_mesh_sub2 = load_model(&format!(
        r#"<mujoco>
  <option gravity="0 0 -9.81"/>
  <asset>
    <mesh name="s" vertex="{verts2}" face="{faces2}"/>
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

    eprintln!("\nMesh sphere (subdivision 2, 162 verts, default=convex):");
    eprintln!("  body_mass[1]    = {:.6e}", m_mesh_sub2.body_mass[1]);
    eprintln!(
        "  mass ratio (mesh/analytic) = {:.6e}",
        m_mesh_sub2.body_mass[1] / analytic_mass
    );

    eprintln!("\n=== Mass Comparison Complete ===\n");
}
