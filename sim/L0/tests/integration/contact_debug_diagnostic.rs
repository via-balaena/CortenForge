//! Diagnostic: multi-body contact behavior for the contact_debug Bevy example.

use sim_mjcf::load_model;

#[test]
fn contact_debug_multi_body_diagnostic() {
    let mjcf = r#"
        <mujoco model="contact_debug">
            <option gravity="0 0 -9.81" timestep="0.002"/>
            <worldbody>
                <geom name="ground" type="plane" size="10 10 0.1" rgba="0.8 0.8 0.8 1"/>
                <body name="sphere1" pos="0 0 3">
                    <freejoint name="sphere1_joint"/>
                    <geom name="sphere1_geom" type="sphere" size="0.4" mass="1.0" rgba="0.2 0.6 1.0 1"/>
                </body>
                <body name="sphere2" pos="0.5 0.3 4">
                    <freejoint name="sphere2_joint"/>
                    <geom name="sphere2_geom" type="sphere" size="0.4" mass="1.0" rgba="0.2 0.8 0.4 1"/>
                </body>
                <body name="sphere3" pos="-0.3 -0.2 5">
                    <freejoint name="sphere3_joint"/>
                    <geom name="sphere3_geom" type="sphere" size="0.4" mass="1.0" rgba="0.8 0.4 0.2 1"/>
                </body>
                <body name="sphere4" pos="0.2 0.5 6">
                    <freejoint name="sphere4_joint"/>
                    <geom name="sphere4_geom" type="sphere" size="0.4" mass="1.0" rgba="0.6 0.2 0.8 1"/>
                </body>
                <body name="box1" pos="-1.5 0 2">
                    <freejoint name="box1_joint"/>
                    <geom name="box1_geom" type="box" size="0.4 0.4 0.4" mass="2.0" rgba="1.0 0.8 0.2 1"/>
                </body>
            </worldbody>
        </mujoco>
    "#;

    let model = load_model(mjcf).expect("load");
    let mut data = model.make_data();
    let _ = data.forward(&model);

    println!("=== INITIAL STATE ===");
    println!(
        "nbody={}, ngeom={}, nq={}, nv={}, solver={:?}",
        model.nbody, model.ngeom, model.nq, model.nv, model.integrator
    );

    let checkpoints = [1, 2, 3, 250, 400, 420, 430, 440, 450, 500, 1000];
    for step in 1..=1000 {
        let _ = data.step(&model);

        if checkpoints.contains(&step) {
            println!("\n=== STEP {} (t={:.4}s) ===", step, data.time);
            println!("contacts: {}", data.ncon);
            for c in &data.contacts {
                println!(
                    "  contact: geom{}↔geom{} normal=[{:.3},{:.3},{:.3}] depth={:.6}",
                    c.geom1, c.geom2, c.normal.x, c.normal.y, c.normal.z, c.depth
                );
            }
            for b in 1..model.nbody {
                let dof = model.body_dof_adr[b];
                let z = data.xpos[b].z;
                let vz = data.qvel[dof + 2];
                let az = data.qacc[dof + 2];
                let bias_z = data.qfrc_bias[dof + 2];
                let passive_z = data.qfrc_passive[dof + 2];
                let applied_z = data.qfrc_applied[dof + 2];
                let actuator_z = data.qfrc_actuator[dof + 2];
                let smooth_z = data.qfrc_smooth[dof + 2];
                let constraint_z = data.qfrc_constraint[dof + 2];
                println!("  body {}: z={:.4} vz={:.4} az={:.4}", b, z, vz, az);
                println!(
                    "    bias_z={:.4} passive_z={:.4} applied_z={:.4} actuator_z={:.4} smooth_z={:.4} constraint_z={:.4}",
                    bias_z, passive_z, applied_z, actuator_z, smooth_z, constraint_z
                );
                let expected_smooth = applied_z + actuator_z + passive_z - bias_z;
                if (smooth_z - expected_smooth).abs() > 0.01 {
                    println!(
                        "    *** DISCREPANCY: smooth_z={:.4} != applied+actuator+passive-bias={:.4} (diff={:.4}) ***",
                        smooth_z,
                        expected_smooth,
                        smooth_z - expected_smooth
                    );
                }
                if b <= 4 && (smooth_z + 9.81).abs() > 1.0 {
                    println!(
                        "    *** WRONG SMOOTH: expected ~-9.81 for mass-1 sphere, got {:.4} ***",
                        smooth_z
                    );
                }
            }
        }
    }
}
