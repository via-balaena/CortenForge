//! Rung 3 of the geometry-fidelity ladder — one part as PHYSICS.
//!
//! The L4 vertebra stops being pure geometry and enters the rigid engine:
//! its mass properties are computed from the mesh (exact Mirtich
//! signed-tetrahedron inertia on the true concave mesh, `inertia="exact"`), it
//! becomes a free rigid body, and it is dropped onto a plane under gravity to
//! confirm it behaves physically (settles, rests on the plane, force-balanced).
//!
//! The exact-inertia routine itself is validated analytically by
//! `sim/L0/tests/integration/exactmeshinertia.rs` (unit cube vs closed form);
//! here we additionally prove exact mode is *active* (not a silent convex-hull
//! fallback) by compiling the vertebra both ways and asserting the concave
//! inertia genuinely differs from the hull.
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

/// mm → m: `BodyParts3D` anatomy is in millimetres; the rigid engine is SI.
const MM_TO_M: f64 = 0.001;
const DROP_H_M: f64 = 0.02;
/// kg/m³ = SOLID cortical bone. A real vertebra is porous (trabecular core +
/// thin cortical shell), so this over-estimates mass ~5-8× (a true L4 is
/// ~30-70 g) — a solid-bone upper bound. Settling is mass-independent, so this
/// choice does not affect the physics conclusion.
const DENSITY: f64 = 1900.0;

/// Total normal+friction contact-constraint force magnitude — at rest this
/// balances the body's weight (Newton's third law).
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

/// L2 speed over a DOF sub-range (keeps linear m/s and angular rad/s separate
/// rather than norming mixed units together).
fn speed(data: &sim_core::Data, skip: usize) -> f64 {
    data.qvel
        .iter()
        .skip(skip)
        .take(3)
        .map(|x| x * x)
        .sum::<f64>()
        .sqrt()
}

/// Freejoint L4 mesh over a plane, with a chosen mesh-inertia mode.
fn scene_mjcf(stl_path: &str, inertia_mode: &str) -> String {
    let s = MM_TO_M;
    format!(
        r#"<mujoco>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <asset>
    <mesh name="l4" file="{stl_path}" scale="{s} {s} {s}" inertia="{inertia_mode}"/>
  </asset>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.1"/>
    <body name="l4" pos="0 0 {DROP_H_M}">
      <freejoint/>
      <geom type="mesh" mesh="l4" density="{DENSITY}"/>
    </body>
  </worldbody>
</mujoco>"#
    )
}

#[test]
#[ignore = "needs a local vertebra mesh via $CF_L4_STL (CC BY-SA asset, not committed)"]
fn l4_settles_on_a_plane_as_a_rigid_body() {
    let path = std::env::var("CF_L4_STL").expect("set $CF_L4_STL to a lumbar vertebra STL");

    // 1. Load + repair, then recenter so the mesh sits just above z=0 in its own
    //    frame (BodyParts3D coords are offset to whole-body position). Move the
    //    AABB min-corner to the origin; the body's MJCF `pos` then lifts it.
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

    let dir = tempfile::tempdir().expect("tempdir");
    let stl = dir.path().join("l4_recentered.stl");
    save_stl(&mesh, &stl, true).expect("save stl");
    let stl_path = stl.to_str().expect("utf-8 temp path");
    // Guard the raw path interpolation into the MJCF XML attribute.
    assert!(
        !stl_path.contains(['"', '<', '>', '&']),
        "temp path is not XML-attribute-safe: {stl_path}"
    );

    // 2. Compile the exact-concave-inertia model, and a convex-hull one purely
    //    to PROVE exact mode is active: a vertebra has a large foramen + gaps
    //    between processes, so the true concave mass MUST be well below the hull.
    let model = load_model(&scene_mjcf(stl_path, "exact")).expect("exact model");
    let hull = load_model(&scene_mjcf(stl_path, "convex")).expect("convex model");
    let (mass, inertia) = (model.body_mass[1], model.body_inertia[1]);
    let hull_mass = hull.body_mass[1];
    println!(
        "[inertia] exact mass={:.2} g  hull mass={:.2} g  (exact/hull={:.2})",
        mass * 1000.0,
        hull_mass * 1000.0,
        mass / hull_mass
    );
    println!(
        "[inertia] exact diag ({:.3e},{:.3e},{:.3e}) kg·m²",
        inertia.x, inertia.y, inertia.z
    );

    assert!(mass > 0.0, "mass must be positive");
    assert!(
        inertia.x > 0.0 && inertia.y > 0.0 && inertia.z > 0.0,
        "principal inertia must be positive-definite, got {inertia:?}"
    );
    // Exact mode is genuinely active (not a silent hull fallback): the concave
    // vertebra encloses much less volume than its convex hull.
    assert!(
        mass < hull_mass * 0.9,
        "exact concave mass ({mass}) should be well below hull mass ({hull_mass}) — \
         else inertia='exact' is not active"
    );

    // 3. Drop the (correct) exact-inertia body and watch it settle.
    let weight = mass * 9.81;
    let mut data = model.make_data();
    data.forward(&model).expect("initial forward"); // populate xipos at release
    let initial_com_z = data.xipos[1].z;

    let n_steps = 1500;
    for step in 0..n_steps {
        data.step(&model).expect("step");
        if step % 300 == 0 || step == n_steps - 1 {
            println!(
                "  t={:.2}s  com_z={:.4}  lin={:.5} ang={:.5}  ncon={}  Fc/mg={:.3}",
                data.time,
                data.xipos[1].z,
                speed(&data, 0),
                speed(&data, 3),
                data.contacts.len(),
                contact_force(&data) / weight
            );
        }
    }

    let (lin, ang) = (speed(&data, 0), speed(&data, 3));
    let final_com_z = data.xipos[1].z;
    let force_ratio = contact_force(&data) / weight;
    let max_pen = data
        .contacts
        .iter()
        .map(|c| c.depth)
        .fold(0.0_f64, f64::max);
    let any_nan = data.qpos.iter().any(|q| q.is_nan());
    println!(
        "[final] com_z={final_com_z:.4} (from {initial_com_z:.4})  lin={lin:.5} ang={ang:.5}  \
         Fc/mg={force_ratio:.3}  max_pen={max_pen:.2e}  nan={any_nan}\n"
    );

    // 4. Physical settling.
    assert!(!any_nan, "simulation diverged (NaN)");
    // Descended under gravity but the COM stayed above the plane...
    assert!(
        final_com_z > 0.0 && final_com_z < initial_com_z,
        "COM should descend yet rest above the plane ({initial_com_z:.4} → {final_com_z:.4})"
    );
    // ...and the contact penetration is bounded (mm-scale), i.e. it is resting
    // ON the plane, not tunnelling through it.
    assert!(
        max_pen < 1e-3,
        "resting penetration should be sub-millimetre, got {max_pen}"
    );
    // Came to rest (linear m/s and angular rad/s checked separately).
    assert!(
        lin < 1e-3 && ang < 1e-3,
        "L4 should come to rest, got lin={lin} ang={ang}"
    );
    // Contact reaction balances weight (Newton's third law).
    assert!(
        (force_ratio - 1.0).abs() < 0.1,
        "contact reaction should balance weight at rest, got Fc/mg={force_ratio}"
    );
}
