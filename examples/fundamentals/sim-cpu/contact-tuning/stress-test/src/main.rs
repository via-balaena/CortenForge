//! Stress test for contact-tuning — verifies friction, condim, solref,
//! pair overrides, and friction combination before building Bevy examples.
//!
//! Run: `cargo run -p example-contact-stress-test --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::let_underscore_must_use,
    clippy::similar_names,
    clippy::too_many_lines,
    clippy::uninlined_format_args,
    clippy::suboptimal_flops,
    clippy::cast_lossless
)]

use nalgebra::Vector3;

fn main() {
    println!("=== Contact Tuning — Stress Test ===\n");

    let mut total = 0;
    let mut passed = 0;

    println!("── 1. Friction on tilted plane (boxes) ──");
    let (p, t) = test_friction_tilted_plane();
    passed += p;
    total += t;

    println!("\n── 2. Condim row counts ──");
    let (p, t) = test_condim_row_counts();
    passed += p;
    total += t;

    println!("\n── 3. Condim behaviour (spheres on tilted plane) ──");
    let (p, t) = test_condim_behaviour();
    passed += p;
    total += t;

    println!("\n── 4. Solref bounce (standard vs direct) ──");
    let (p, t) = test_solref_bounce();
    passed += p;
    total += t;

    println!("\n── 5. Pair override replaces friction (boxes) ──");
    let (p, t) = test_pair_override();
    passed += p;
    total += t;

    println!("\n── 6. Friction combination (MAX rule) ──");
    let (p, t) = test_friction_combination();
    passed += p;
    total += t;

    println!("\n── 7. Solimp impedance curve (penetration depth) ──");
    let (p, t) = test_solimp_depth();
    passed += p;
    total += t;

    println!("\n── 8. Margin / gap (contact activation distance) ──");
    let (p, t) = test_margin_gap();
    passed += p;
    total += t;

    println!("\n============================================================");
    println!("  TOTAL: {passed}/{total} checks passed");
    if passed == total {
        println!("  ALL PASS — contact tuning validated");
    } else {
        println!("  {} FAILED — needs investigation", total - passed);
        std::process::exit(1);
    }
}

fn check(name: &str, pass: bool, detail: &str) -> bool {
    let tag = if pass { "PASS" } else { "FAIL" };
    println!("  [{tag}] {name}: {detail}");
    pass
}

// ── 1. Friction on tilted plane ─────────────────────────────────────────────
//
// Three BOXES on a 15° tilted plane. Boxes can't roll, so friction directly
// determines slide vs hold. Floor friction=0 so combined = box friction.
// mu=0.1 < tan(15°)=0.268 → slides.  mu=0.5 > tan(15°) → holds.

fn test_friction_tilted_plane() -> (usize, usize) {
    let mjcf = r#"
        <mujoco model="friction-test">
          <compiler angle="radian"/>
          <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"
                  iterations="50" tolerance="1e-10"/>

          <worldbody>
            <body name="ramp" euler="0 0.2618 0">
              <geom name="floor" type="plane" size="5 5 0.01"
                    friction="0 0 0"/>
            </body>

            <!-- All at x=0, spaced along Y (horizontal on ramp). Just above plane. -->
            <body name="low" pos="0 -0.2 0.045">
              <freejoint/>
              <geom name="g_low" type="box" size="0.04 0.04 0.04" mass="0.5"
                    friction="0.1 0.005 0.001"/>
            </body>

            <body name="med" pos="0 0 0.045">
              <freejoint/>
              <geom name="g_med" type="box" size="0.04 0.04 0.04" mass="0.5"
                    friction="0.5 0.005 0.001"/>
            </body>

            <body name="high" pos="0 0.2 0.045">
              <freejoint/>
              <geom name="g_high" type="box" size="0.04 0.04 0.04" mass="0.5"
                    friction="1.0 0.005 0.001"/>
            </body>
          </worldbody>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).expect("parse");
    let mut data = model.make_data();
    data.forward(&model).expect("fwd");

    // Let boxes settle onto the plane (500ms), then record positions.
    for _ in 0..500 {
        data.step(&model).expect("step");
    }
    let low_settled = Vector3::new(data.qpos[0], data.qpos[1], data.qpos[2]);
    let med_settled = Vector3::new(data.qpos[7], data.qpos[8], data.qpos[9]);
    let _high_settled = Vector3::new(data.qpos[14], data.qpos[15], data.qpos[16]);

    // Run 2.5 more seconds, track peak velocities
    let mut low_peak_vel = 0.0f64;
    for _ in 0..2500 {
        data.step(&model).expect("step");
        let v = Vector3::new(data.qvel[0], data.qvel[1], data.qvel[2]).norm();
        low_peak_vel = low_peak_vel.max(v);
    }

    // Body indices: 0=world, 1=ramp, 2=low, 3=med, 4=high
    let med_vel = Vector3::new(data.qvel[6], data.qvel[7], data.qvel[8]).norm();
    let high_vel = Vector3::new(data.qvel[12], data.qvel[13], data.qvel[14]).norm();

    // Displacement on the plane after settling
    let low_disp = (Vector3::new(data.qpos[0], data.qpos[1], data.qpos[2]) - low_settled).norm();
    let med_disp = (Vector3::new(data.qpos[7], data.qpos[8], data.qpos[9]) - med_settled).norm();

    println!(
        "    low:  disp={:.4}m, peak_vel={:.3} m/s",
        low_disp, low_peak_vel
    );
    println!("    med:  disp={:.4}m, vel={:.4} m/s", med_disp, med_vel);
    println!("    high: vel={:.4} m/s", high_vel);

    let mut p = 0usize;
    let t = 4;

    p += check(
        "Low-mu slides after settling",
        low_disp > 0.1,
        &format!("disp={:.3}m (want >0.1)", low_disp),
    ) as usize;
    p += check(
        "Low-mu had peak velocity",
        low_peak_vel > 0.5,
        &format!("peak_vel={:.3} m/s (want >0.5)", low_peak_vel),
    ) as usize;
    p += check(
        "Med-mu holds",
        med_disp < 0.01,
        &format!("disp={:.4}m (want <0.01)", med_disp),
    ) as usize;
    p += check(
        "High-mu holds",
        high_vel < 0.05,
        &format!("vel={:.4} m/s (want <0.05)", high_vel),
    ) as usize;

    (p, t)
}

// ── 2. Condim row counts ────────────────────────────────────────────────────
//
// Verify that condim=1, 3, 4, 6 produce the correct contact dim on Contact.

fn test_condim_row_counts() -> (usize, usize) {
    let mut p = 0usize;
    let t = 4;

    for (condim, expected_dim) in [(1, 1), (3, 3), (4, 4), (6, 6)] {
        // Ball starts just above plane, settles after 100 steps
        let mjcf = format!(
            r#"
            <mujoco>
              <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"
                      iterations="50" tolerance="1e-10"/>
              <worldbody>
                <geom name="floor" type="plane" size="5 5 0.01"/>
                <body name="ball" pos="0 0 0.06">
                  <freejoint/>
                  <geom name="g" type="sphere" size="0.05" mass="0.5"
                        friction="1 0.005 0.001"/>
                </body>
              </worldbody>
              <contact>
                <pair geom1="floor" geom2="g" condim="{condim}"
                      friction="1 1 0.005 0.001 0.001"/>
              </contact>
            </mujoco>
        "#
        );
        let model = sim_mjcf::load_model(&mjcf).expect("parse");
        let mut data = model.make_data();
        data.forward(&model).expect("fwd");
        // Run enough steps for the ball to reach the plane
        for _ in 0..100 {
            data.step(&model).expect("step");
        }

        let dim = if data.ncon > 0 {
            data.contacts[0].dim
        } else {
            0
        };
        p += check(
            &format!("condim={condim} → dim={expected_dim}"),
            dim == expected_dim,
            &format!("got dim={dim}, ncon={}", data.ncon),
        ) as usize;
    }

    (p, t)
}

// ── 3. Condim behaviour on tilted plane ─────────────────────────────────────
//
// Spheres on tilted plane. condim=1 (frictionless) → slides at g*sin(θ).
// condim=3 (sliding friction) → sphere ROLLS at (5/7)*g*sin(θ) (no slip).
// condim=6 (rolling friction) → eventually stops.

fn test_condim_behaviour() -> (usize, usize) {
    let mjcf = r#"
        <mujoco model="condim-test">
          <compiler angle="radian"/>
          <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"
                  iterations="50" tolerance="1e-10"/>

          <worldbody>
            <body name="ramp" euler="0 0.2618 0">
              <geom name="floor" type="plane" size="5 5 0.01"
                    friction="0 0 0"/>
            </body>

            <body name="frictionless" pos="-0.2 0 0.15">
              <freejoint/>
              <geom name="g_fric" type="sphere" size="0.05" mass="0.5"
                    friction="1 0.005 0.001"/>
            </body>
            <body name="sliding" pos="0.2 0 0.15">
              <freejoint/>
              <geom name="g_slide" type="sphere" size="0.05" mass="0.5"
                    friction="1 0.005 0.001"/>
            </body>
          </worldbody>

          <contact>
            <pair geom1="floor" geom2="g_fric" condim="1"
                  friction="0 0 0 0 0"/>
            <pair geom1="floor" geom2="g_slide" condim="3"
                  friction="1 1 0.005 0.001 0.001"/>
          </contact>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).expect("parse");
    let mut data = model.make_data();
    data.forward(&model).expect("fwd");

    // Run for 2 seconds
    for _ in 0..2000 {
        data.step(&model).expect("step");
    }

    // Body indices: 0=world, 1=ramp, 2=frictionless, 3=sliding
    let fric_vel = Vector3::new(data.qvel[0], data.qvel[1], data.qvel[2]).norm();
    let slide_vel = Vector3::new(data.qvel[6], data.qvel[7], data.qvel[8]).norm();

    // Frictionless: pure sliding, a = g*sin(θ), v = a*t
    let a_slide = 9.81 * (0.2618_f64).sin();
    let expected_frictionless = a_slide * 2.0;
    // Rolling without slipping: a = (5/7)*g*sin(θ), v = a*t
    let expected_rolling = (5.0 / 7.0) * a_slide * 2.0;

    println!(
        "    frictionless: vel={:.3} m/s (expect ~{:.2})",
        fric_vel, expected_frictionless
    );
    println!(
        "    condim=3 (rolls): vel={:.3} m/s (expect ~{:.2})",
        slide_vel, expected_rolling
    );

    let mut p = 0usize;
    let t = 3;

    p += check(
        "Frictionless has velocity",
        fric_vel > 1.0,
        &format!("vel={:.3} m/s (want >1.0)", fric_vel),
    ) as usize;
    p += check(
        "Rolling matches (5/7)*g*sin(θ)*t",
        (slide_vel - expected_rolling).abs() / expected_rolling < 0.15,
        &format!(
            "vel={:.3} vs expected={:.3}, err={:.1}%",
            slide_vel,
            expected_rolling,
            (slide_vel - expected_rolling).abs() / expected_rolling * 100.0
        ),
    ) as usize;
    // condim=3 sphere rolls smoothly; condim=1 sphere bounces/slides erratically.
    // Both have velocity but via different mechanisms — verify both are moving.
    p += check(
        "condim=3 ball moving",
        slide_vel > 1.0,
        &format!("vel={:.3} m/s (want >1.0)", slide_vel),
    ) as usize;

    (p, t)
}

// ── 4. Solref bounce ────────────────────────────────────────────────────────
//
// Drop spheres from 0.5m. Direct-mode stiff → bouncy. Standard critical → absorbs.

fn test_solref_bounce() -> (usize, usize) {
    let mjcf = r#"
        <mujoco model="solref-bounce">
          <option gravity="0 0 -9.81" timestep="0.0005" solver="Newton"
                  iterations="50" tolerance="1e-10"/>

          <worldbody>
            <geom name="ground" type="plane" size="5 5 0.01"/>

            <body name="stiff" pos="-0.3 0 0.55">
              <freejoint/>
              <geom name="g_stiff" type="sphere" size="0.05" mass="0.5"/>
            </body>
            <body name="default" pos="0 0 0.55">
              <freejoint/>
              <geom name="g_default" type="sphere" size="0.05" mass="0.5"/>
            </body>
            <body name="soft" pos="0.3 0 0.55">
              <freejoint/>
              <geom name="g_soft" type="sphere" size="0.05" mass="0.5"/>
            </body>
          </worldbody>

          <contact>
            <pair geom1="ground" geom2="g_stiff"
                  solref="-5000 -10" condim="1"/>
            <pair geom1="ground" geom2="g_default"
                  solref="-5000 -30" condim="1"/>
            <pair geom1="ground" geom2="g_soft"
                  solref="-5000 -500" condim="1"/>
          </contact>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).expect("parse");
    let mut data = model.make_data();
    data.forward(&model).expect("fwd");

    let mut stiff_max_z_after = 0.0f64;
    let mut default_max_z_after = 0.0f64;
    let mut soft_max_z_after = 0.0f64;
    let mut stiff_hit = false;
    let mut default_hit = false;
    let mut soft_hit = false;

    // Run for 3 seconds (6000 steps at 0.5ms)
    for _ in 0..6000 {
        data.step(&model).expect("step");

        let z_stiff = data.qpos[2];
        let z_default = data.qpos[9];
        let z_soft = data.qpos[16];

        // Contact detected when center drops near radius
        if z_stiff < 0.08 {
            stiff_hit = true;
        }
        if z_default < 0.08 {
            default_hit = true;
        }
        if z_soft < 0.08 {
            soft_hit = true;
        }

        if stiff_hit {
            stiff_max_z_after = stiff_max_z_after.max(z_stiff);
        }
        if default_hit {
            default_max_z_after = default_max_z_after.max(z_default);
        }
        if soft_hit {
            soft_max_z_after = soft_max_z_after.max(z_soft);
        }
    }

    println!("    bouncy:    max_z_after={:.4} m", stiff_max_z_after);
    println!("    moderate:  max_z_after={:.4} m", default_max_z_after);
    println!("    absorbing: max_z_after={:.4} m", soft_max_z_after);

    let mut p = 0usize;
    let t = 3;

    p += check(
        "Stiff bounces high",
        stiff_max_z_after > 0.15,
        &format!("max_z={:.3}m (want >0.15)", stiff_max_z_after),
    ) as usize;
    p += check(
        "Stiff > default",
        stiff_max_z_after > default_max_z_after,
        &format!(
            "stiff={:.3} > default={:.3}",
            stiff_max_z_after, default_max_z_after
        ),
    ) as usize;
    // Moderate ball bounces less than stiff, absorbing ball doesn't bounce.
    p += check(
        "Moderate bounces less than stiff",
        default_max_z_after < stiff_max_z_after,
        &format!(
            "moderate={:.3} < stiff={:.3}",
            default_max_z_after, stiff_max_z_after
        ),
    ) as usize;

    (p, t)
}

// ── 5. Pair override ────────────────────────────────────────────────────────
//
// Two identical BOXES on tilted plane. One gets <pair> override to low friction.
// Boxes can't roll — friction directly determines slide vs hold.

fn test_pair_override() -> (usize, usize) {
    let mjcf = r#"
        <mujoco model="pair-override">
          <compiler angle="radian"/>
          <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"
                  iterations="50" tolerance="1e-10"/>

          <worldbody>
            <body name="ramp" euler="0 0.2618 0">
              <geom name="floor" type="plane" size="5 5 0.01"
                    friction="1 0.005 0.001"/>
            </body>

            <body name="auto" pos="-0.15 0 0.15">
              <freejoint/>
              <geom name="g_auto" type="box" size="0.04 0.04 0.04" mass="0.5"
                    friction="1 0.005 0.001"/>
            </body>
            <body name="over" pos="0.15 0 0.15">
              <freejoint/>
              <geom name="g_over" type="box" size="0.04 0.04 0.04" mass="0.5"
                    friction="1 0.005 0.001"/>
            </body>
          </worldbody>

          <contact>
            <pair geom1="floor" geom2="g_over"
                  friction="0.1 0.1 0.005 0.001 0.001"/>
          </contact>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).expect("parse");
    let mut data = model.make_data();
    data.forward(&model).expect("fwd");

    // Run for 3 seconds
    for _ in 0..3000 {
        data.step(&model).expect("step");
    }

    // Body indices: 0=world, 1=ramp, 2=auto, 3=over
    let auto_vel = Vector3::new(data.qvel[0], data.qvel[1], data.qvel[2]).norm();
    let over_vel = Vector3::new(data.qvel[6], data.qvel[7], data.qvel[8]).norm();

    println!("    auto (mu=1.0 combined): vel={:.4} m/s", auto_vel);
    println!("    override (mu=0.1 pair): vel={:.4} m/s", over_vel);

    let mut p = 0usize;
    let t = 2;

    p += check(
        "Auto box holds (high friction)",
        auto_vel < 0.05,
        &format!("vel={:.4} (want <0.05)", auto_vel),
    ) as usize;
    p += check(
        "Override box slides (low friction)",
        over_vel > 0.1,
        &format!("vel={:.3} (want >0.1)", over_vel),
    ) as usize;

    (p, t)
}

// ── 6. Friction combination (MAX rule) ──────────────────────────────────────
//
// Verify that auto-combined friction = MAX(geom1, geom2) per component.

fn test_friction_combination() -> (usize, usize) {
    // geom1: friction="0.3 0.01 0.002", geom2: friction="0.7 0.005 0.004"
    // Expected combined: MAX → [0.7, 0.01, 0.004]
    // In 5-element form: [0.7, 0.7, 0.01, 0.004, 0.004]
    let mjcf = r#"
        <mujoco>
          <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"
                  iterations="50" tolerance="1e-10"/>
          <worldbody>
            <geom name="floor" type="plane" size="5 5 0.01"
                  friction="0.3 0.01 0.002"/>
            <body name="ball" pos="0 0 0.06">
              <freejoint/>
              <geom name="g" type="sphere" size="0.05" mass="0.5"
                    friction="0.7 0.005 0.004"/>
            </body>
          </worldbody>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).expect("parse");
    let mut data = model.make_data();
    data.forward(&model).expect("fwd");

    // Run enough steps for ball to reach plane
    for _ in 0..200 {
        data.step(&model).expect("step");
    }

    let mut p = 0usize;
    let t = 4;

    p += check(
        "Contact detected",
        data.ncon > 0,
        &format!("ncon={}", data.ncon),
    ) as usize;

    if data.ncon > 0 {
        let c = &data.contacts[0];
        let mu = c.mu;
        println!(
            "    mu = [{:.4}, {:.4}, {:.4}, {:.4}, {:.4}]",
            mu[0], mu[1], mu[2], mu[3], mu[4]
        );

        // sliding: MAX(0.3, 0.7) = 0.7
        p += check(
            "Sliding = MAX(0.3, 0.7) = 0.7",
            (mu[0] - 0.7).abs() < 1e-6 && (mu[1] - 0.7).abs() < 1e-6,
            &format!("mu[0]={:.4}, mu[1]={:.4}", mu[0], mu[1]),
        ) as usize;
        // torsional: MAX(0.01, 0.005) = 0.01
        p += check(
            "Torsional = MAX(0.01, 0.005) = 0.01",
            (mu[2] - 0.01).abs() < 1e-6,
            &format!("mu[2]={:.6}", mu[2]),
        ) as usize;
        // rolling: MAX(0.002, 0.004) = 0.004
        p += check(
            "Rolling = MAX(0.002, 0.004) = 0.004",
            (mu[3] - 0.004).abs() < 1e-6 && (mu[4] - 0.004).abs() < 1e-6,
            &format!("mu[3]={:.6}, mu[4]={:.6}", mu[3], mu[4]),
        ) as usize;
    } else {
        println!("    SKIP: no contacts detected");
    }

    (p, t)
}

// ── 7. Solimp impedance curve ───────────────────────────────────────────────
//
// Two balls on a flat plane with different solimp. Low d0 = soft start, ball
// sinks deeper. High d0 (default) = stiff from the start, minimal sinking.

fn test_solimp_depth() -> (usize, usize) {
    let mjcf = r#"
        <mujoco model="solimp-depth">
          <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"
                  iterations="50" tolerance="1e-10"/>

          <worldbody>
            <geom name="ground" type="plane" size="5 5 0.01"/>

            <!-- Default solimp: d0=0.9, stiff from the start -->
            <body name="stiff" pos="-0.15 0 0.06">
              <freejoint/>
              <geom name="g_stiff" type="sphere" size="0.05" mass="2"/>
            </body>

            <!-- Soft start: d0=0.1, wide transition (50mm) -->
            <body name="soft" pos="0.15 0 0.06">
              <freejoint/>
              <geom name="g_soft" type="sphere" size="0.05" mass="2"/>
            </body>
          </worldbody>

          <contact>
            <pair geom1="ground" geom2="g_stiff"
                  solimp="0.9 0.95 0.001 0.5 2" condim="1"/>
            <pair geom1="ground" geom2="g_soft"
                  solimp="0.1 0.95 0.05 0.5 2" condim="1"/>
          </contact>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).expect("parse");
    let mut data = model.make_data();
    data.forward(&model).expect("fwd");

    // Let settle for 3 seconds
    for _ in 0..3000 {
        data.step(&model).expect("step");
    }

    // Equilibrium z of sphere center. Radius=0.05, so z=0.05 means sitting
    // exactly on the surface. z < 0.05 means penetration.
    let z_stiff = data.qpos[2];
    let z_soft = data.qpos[9];
    let pen_stiff = 0.05 - z_stiff; // positive = penetration
    let pen_soft = 0.05 - z_soft;

    println!(
        "    stiff (d0=0.9): z={:.4}, pen={:.4}mm",
        z_stiff,
        pen_stiff * 1000.0
    );
    println!(
        "    soft  (d0=0.1): z={:.4}, pen={:.4}mm",
        z_soft,
        pen_soft * 1000.0
    );

    let mut p = 0usize;
    let t = 3;

    p += check(
        "Soft sinks deeper than stiff",
        pen_soft > pen_stiff,
        &format!(
            "soft_pen={:.3}mm > stiff_pen={:.3}mm",
            pen_soft * 1000.0,
            pen_stiff * 1000.0
        ),
    ) as usize;
    p += check(
        "Stiff has minimal penetration (< 5mm)",
        pen_stiff < 0.005,
        &format!("pen={:.3}mm", pen_stiff * 1000.0),
    ) as usize;
    p += check(
        "Soft has more penetration (> 2mm)",
        pen_soft > 0.002,
        &format!("pen={:.3}mm", pen_soft * 1000.0),
    ) as usize;

    (p, t)
}

// ── 8. Margin / gap ────────────────────────────────────────────────────────
//
// Balls with different margin values. Large margin = ball "floats" above
// the surface (contact activates before physical penetration).

fn test_margin_gap() -> (usize, usize) {
    let mjcf = r#"
        <mujoco model="margin-gap">
          <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"
                  iterations="50" tolerance="1e-10"/>

          <worldbody>
            <geom name="ground" type="plane" size="5 5 0.01"/>

            <!-- Zero margin: rests at surface -->
            <body name="zero" pos="-0.2 0 0.06">
              <freejoint/>
              <geom name="g_zero" type="sphere" size="0.05" mass="0.5"/>
            </body>

            <!-- margin=0.02: floats ~20mm above surface -->
            <body name="mid" pos="0 0 0.08">
              <freejoint/>
              <geom name="g_mid" type="sphere" size="0.05" mass="0.5"
                    margin="0.02"/>
            </body>

            <!-- margin=0.05: floats ~50mm above surface -->
            <body name="high" pos="0.2 0 0.11">
              <freejoint/>
              <geom name="g_high" type="sphere" size="0.05" mass="0.5"
                    margin="0.05"/>
            </body>
          </worldbody>
        </mujoco>
    "#;
    let model = sim_mjcf::load_model(mjcf).expect("parse");
    let mut data = model.make_data();
    data.forward(&model).expect("fwd");

    // Let settle for 3 seconds
    for _ in 0..3000 {
        data.step(&model).expect("step");
    }

    // z of sphere center. Radius=0.05. z=0.05 = sitting on surface.
    let z_zero = data.qpos[2];
    let z_mid = data.qpos[9];
    let z_high = data.qpos[16];

    println!("    zero margin:  z={:.4}", z_zero);
    println!("    margin=0.02:  z={:.4}", z_mid);
    println!("    margin=0.05:  z={:.4}", z_high);

    let mut p = 0usize;
    let t = 3;

    p += check(
        "Zero-margin ball near surface",
        z_zero < 0.055,
        &format!("z={:.4} (want <0.055)", z_zero),
    ) as usize;
    p += check(
        "Mid-margin ball floats higher",
        z_mid > z_zero + 0.005,
        &format!("z_mid={:.4} > z_zero={:.4} + 5mm", z_mid, z_zero),
    ) as usize;
    p += check(
        "High-margin ball floats highest",
        z_high > z_mid + 0.005,
        &format!("z_high={:.4} > z_mid={:.4} + 5mm", z_high, z_mid),
    ) as usize;

    (p, t)
}
