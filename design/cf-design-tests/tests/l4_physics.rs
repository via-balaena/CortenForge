//! Rung 3 of the geometry-fidelity ladder — one part as PHYSICS.
//!
//! The L4 vertebra stops being pure geometry and enters the rigid engine:
//! its mass properties are computed from the mesh (exact Mirtich
//! signed-tetrahedron inertia, via MJCF `exactmeshinertia`), it becomes a
//! free rigid body, and it is dropped onto a plane under gravity to confirm it
//! behaves physically (settles, no fall-through, no divergence).
//!
//! Env-gated + license-clean exactly like the rung-2 round-trip: `#[ignore]` +
//! `$CF_L4_STL` (the CC BY-SA mesh is not committed). Run with:
//!
//! ```text
//! CF_L4_STL=/path/to/L4.stl cargo test -p cf-design-tests \
//!   --release --test l4_physics -- --ignored --nocapture
//! ```

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use cf_geometry::Aabb;
use mesh_io::{load_stl, save_stl};
use mesh_repair::{RepairParams, repair_mesh};
use sim_core::ConstraintType;
use sim_mjcf::load_model;

/// Total normal+friction contact-constraint force magnitude at the current step
/// — at rest this balances the body's weight (Newton's third law).
fn contact_force(data: &sim_core::Data) -> f64 {
    data.efc_type
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
        .map(|(i, _)| data.efc_force[i].abs())
        .sum()
}

/// mm → m: `BodyParts3D` anatomy is in millimetres; the rigid engine is SI.
const MM_TO_M: f64 = 0.001;

#[test]
#[ignore = "needs a local vertebra mesh via $CF_L4_STL (CC BY-SA asset, not committed)"]
fn l4_settles_on_a_plane_as_a_rigid_body() {
    let path = std::env::var("CF_L4_STL").expect("set $CF_L4_STL to a lumbar vertebra STL");

    // 1. Load + repair, then recenter so the mesh sits just above z=0 in its own
    //    frame (BodyParts3D coords are offset to whole-body position). We move
    //    the AABB min-corner to the origin; the body's MJCF `pos` then lifts it.
    let mut mesh = load_stl(&path).expect("load L4");
    let repair = repair_mesh(&mut mesh, &RepairParams::for_scans());
    println!("[repair] welded {} verts", repair.vertices_welded);
    let bbox = Aabb::from_points(mesh.vertices.iter());
    let shift = -bbox.min.coords;
    for v in &mut mesh.vertices {
        *v += shift;
    }
    let ext = bbox.max - bbox.min; // mm
    println!(
        "[L4] {} faces, extent ({:.1},{:.1},{:.1}) mm",
        mesh.faces.len(),
        ext.x,
        ext.y,
        ext.z
    );

    // Write the recentered mesh to a temp STL the MJCF can reference by path.
    let dir = tempfile::tempdir().expect("tempdir");
    let stl = dir.path().join("l4_recentered.stl");
    save_stl(&mesh, &stl, true).expect("save stl");

    // 2. Build the scene: freejoint L4 mesh body dropped 2 cm above a plane,
    //    exact concave inertia, cortical-bone density.
    let drop_h_m = 0.02;
    // kg/m^3 = SOLID cortical bone. A real vertebra is porous (trabecular core
    // + thin cortical shell), so this over-estimates mass ~5-8x (a true L4 is
    // ~30-70 g) — it is a solid-bone upper bound. Settling is mass-independent,
    // so this choice does not affect the physics conclusion.
    let density = 1900.0;
    let mjcf = format!(
        r#"<mujoco>
  <compiler exactmeshinertia="true"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <asset>
    <mesh name="l4" file="{path}" scale="{s} {s} {s}"/>
  </asset>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.1"/>
    <body name="l4" pos="0 0 {drop_h_m}">
      <freejoint/>
      <geom type="mesh" mesh="l4" density="{density}"/>
    </body>
  </worldbody>
</mujoco>"#,
        path = stl.display(),
        s = MM_TO_M,
    );
    let model = load_model(&mjcf).expect("model should load");

    // 3. Mass properties from the mesh.
    let mass = model.body_mass[1];
    let inertia = model.body_inertia[1];
    println!(
        "[mass] {:.4} g   inertia diag ({:.3e},{:.3e},{:.3e}) kg·m²",
        mass * 1000.0,
        inertia.x,
        inertia.y,
        inertia.z
    );

    // 4. Drop it and watch it settle.
    let weight = mass * 9.81;
    let mut data = model.make_data();
    let n_steps = 1500;
    let mut initial_com_z = f64::NAN;
    for step in 0..n_steps {
        data.step(&model).expect("step");
        if step == 0 {
            initial_com_z = data.xipos[1].z;
        }
        if step % 300 == 0 || step == n_steps - 1 {
            let vnorm = data.qvel.iter().map(|v| v * v).sum::<f64>().sqrt();
            println!(
                "  t={:.2}s  com_z={:.4}  |qvel|={:.5}  ncon={}  Fc/mg={:.3}",
                data.time,
                data.xipos[1].z,
                vnorm,
                data.contacts.len(),
                contact_force(&data) / weight
            );
        }
    }
    let final_v = data.qvel.iter().map(|v| v * v).sum::<f64>().sqrt();
    let final_com_z = data.xipos[1].z;
    let force_ratio = contact_force(&data) / weight;
    let any_nan = data.qpos.iter().any(|q| q.is_nan());
    println!(
        "[final] com_z={final_com_z:.4}  |qvel|={final_v:.5}  Fc/mg={force_ratio:.3}  nan={any_nan}\n"
    );

    // Well-posed rigid body: positive mass and positive-definite inertia (the
    // exact-mesh-inertia routine itself is validated analytically by
    // sim/L0/tests/integration/exactmeshinertia.rs — leaned on here, not
    // re-proven).
    assert!(mass > 0.0, "mass must be positive");
    assert!(
        inertia.x > 0.0 && inertia.y > 0.0 && inertia.z > 0.0,
        "principal inertia must be positive-definite, got {inertia:?}"
    );

    // Physical settling: no divergence, the COM descended under gravity but the
    // body did not fall through the plane, it came to rest, and the contact
    // reaction balances gravity (Newton's third law) at steady state.
    assert!(!any_nan, "simulation diverged (NaN)");
    assert!(
        final_com_z > 0.0 && final_com_z < initial_com_z,
        "L4 COM should descend under gravity yet rest above the plane \
         (initial {initial_com_z:.4} → final {final_com_z:.4})"
    );
    assert!(
        final_v < 1e-3,
        "L4 should come to rest, got |qvel|={final_v}"
    );
    assert!(
        (force_ratio - 1.0).abs() < 0.1,
        "contact reaction should balance weight at rest, got Fc/mg={force_ratio}"
    );
}
