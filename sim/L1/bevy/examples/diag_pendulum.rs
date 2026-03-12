//! Quick diagnostic: does n_link_pendulum(1) produce correct MuJoCo-style FK?
//! Run with: cargo run -p sim-bevy --example diag_pendulum --release

use sim_core::Model;
use std::f64::consts::PI;

fn main() {
    let model = Model::n_link_pendulum(1, 1.0, 1.0);
    let mut data = model.make_data();
    data.qpos[0] = PI / 3.0;

    println!(
        "model: nbody={}, njnt={}, nq={}, nv={}",
        model.nbody, model.njnt, model.nq, model.nv
    );
    println!(
        "body_pos[1]: [{:.3}, {:.3}, {:.3}]",
        model.body_pos[0].x, model.body_pos[0].y, model.body_pos[0].z
    );
    println!(
        "body_ipos[1]: [{:.3}, {:.3}, {:.3}]",
        model.body_ipos[0].x, model.body_ipos[0].y, model.body_ipos[0].z
    );
    println!(
        "jnt_pos[0]: [{:.3}, {:.3}, {:.3}]",
        model.jnt_pos[0].x, model.jnt_pos[0].y, model.jnt_pos[0].z
    );
    println!();

    match data.forward(&model) {
        Ok(()) => println!("forward() OK"),
        Err(e) => {
            println!("forward() FAILED: {e:?}");
            return;
        }
    }

    println!(
        "qpos[0]={:.4} ({:.1}°)",
        data.qpos[0],
        data.qpos[0].to_degrees()
    );
    println!(
        "xpos[1]  (body frame): [{:.4}, {:.4}, {:.4}]",
        data.xpos[1].x, data.xpos[1].y, data.xpos[1].z
    );
    println!(
        "xipos[1] (COM):        [{:.4}, {:.4}, {:.4}]",
        data.xipos[1].x, data.xipos[1].y, data.xipos[1].z
    );
    println!();

    // Expected: xpos at (0,0,0), xipos at (-sin(60°), 0, -cos(60°)) = (-0.866, 0, -0.5)
    let expected_x = -(PI / 3.0).sin();
    let expected_z = -(PI / 3.0).cos();
    println!(
        "Expected xipos: [{:.4}, 0.0000, {:.4}]",
        expected_x, expected_z
    );
    println!();

    for i in 0..5 {
        match data.step(&model) {
            Ok(()) => {}
            Err(e) => {
                println!("step {i} FAILED: {e:?}");
                return;
            }
        }
        println!(
            "step {:2}: qpos={:+.6}, qvel={:+.6}, qacc={:+.6}  xipos=[{:.3},{:.3},{:.3}]",
            i,
            data.qpos[0],
            data.qvel[0],
            data.qacc[0],
            data.xipos[1].x,
            data.xipos[1].y,
            data.xipos[1].z
        );
    }
}
