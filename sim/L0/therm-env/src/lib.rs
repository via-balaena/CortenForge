//! `ThermCircuitEnv`: typed builder producing `SimEnv`/`VecEnv` from
//! thermodynamic circuit descriptions.
//!
//! # Phase 1 — MJCF generation
//!
//! [`generate_mjcf`] produces a minimal `MuJoCo` XML string for N 1-DOF
//! particles with M zero-gain control actuators.  The resulting model
//! has `nq = nv = n_particles` and `nu = n_ctrl`.

#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::fmt::Write as _;

/// Generate a minimal MJCF XML string for a thermodynamic circuit.
///
/// Produces `n_particles` bodies, each with a single slide joint (`x0`…`x{N-1}`),
/// and `n_ctrl` zero-gain actuators (`ctrl_0`…`ctrl_{M-1}`).  Zero-gain
/// actuators exist only to allocate `data.ctrl` slots — they produce zero
/// force regardless of the control input.
///
/// # Panics
///
/// This function does not panic.  Invalid inputs (e.g. `n_ctrl > n_particles`)
/// produce valid but degenerate MJCF that the caller may reject.
#[must_use]
pub fn generate_mjcf(n_particles: usize, n_ctrl: usize, timestep: f64) -> String {
    let mut xml = format!(
        r#"<mujoco model="therm-circuit-{n_particles}">
  <option timestep="{timestep}" gravity="0 0 0" integrator="Euler">
    <flag contact="disable"/>
  </option>
  <worldbody>"#
    );

    // fmt::Write for String is infallible — .ok() silences the must-use lint
    for i in 0..n_particles {
        write!(
            xml,
            r#"
    <body name="p{i}">
      <joint name="x{i}" type="slide" axis="1 0 0" damping="0"/>
      <geom type="sphere" size="0.05" mass="1"/>
    </body>"#
        )
        .ok();
    }

    xml.push_str("\n  </worldbody>");

    if n_ctrl > 0 {
        xml.push_str("\n  <actuator>");
        for i in 0..n_ctrl {
            write!(
                xml,
                r#"
    <general name="ctrl_{i}" joint="x{i}" gainprm="0" biasprm="0 0 0"
             ctrllimited="true" ctrlrange="0 10"/>"#
            )
            .ok();
        }
        xml.push_str("\n  </actuator>");
    }

    xml.push_str("\n</mujoco>\n");
    xml
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;

    // ── Parsing ──────────────────────────────────────────────────────

    #[test]
    fn mjcf_parses_1_particle() {
        let xml = generate_mjcf(1, 1, 0.001);
        sim_mjcf::load_model(&xml).unwrap();
    }

    #[test]
    fn mjcf_parses_2_particles() {
        let xml = generate_mjcf(2, 1, 0.001);
        sim_mjcf::load_model(&xml).unwrap();
    }

    #[test]
    fn mjcf_parses_8_particles() {
        let xml = generate_mjcf(8, 4, 0.001);
        sim_mjcf::load_model(&xml).unwrap();
    }

    // ── Dimensions (nq, nv, nu) ──────────────────────────────────────

    #[test]
    fn dimensions_1_particle_1_ctrl() {
        let xml = generate_mjcf(1, 1, 0.001);
        let model = sim_mjcf::load_model(&xml).unwrap();
        assert_eq!(model.nq, 1, "nq");
        assert_eq!(model.nv, 1, "nv");
        assert_eq!(model.nu, 1, "nu");
    }

    #[test]
    fn dimensions_2_particles_1_ctrl() {
        let xml = generate_mjcf(2, 1, 0.001);
        let model = sim_mjcf::load_model(&xml).unwrap();
        assert_eq!(model.nq, 2, "nq");
        assert_eq!(model.nv, 2, "nv");
        assert_eq!(model.nu, 1, "nu");
    }

    #[test]
    fn dimensions_8_particles_4_ctrl() {
        let xml = generate_mjcf(8, 4, 0.001);
        let model = sim_mjcf::load_model(&xml).unwrap();
        assert_eq!(model.nq, 8, "nq");
        assert_eq!(model.nv, 8, "nv");
        assert_eq!(model.nu, 4, "nu");
    }

    #[test]
    fn dimensions_no_actuators() {
        let xml = generate_mjcf(3, 0, 0.001);
        let model = sim_mjcf::load_model(&xml).unwrap();
        assert_eq!(model.nq, 3, "nq");
        assert_eq!(model.nv, 3, "nv");
        assert_eq!(model.nu, 0, "nu");
    }

    // ── Naming ───────────────────────────────────────────────────────

    #[test]
    fn joint_names() {
        let xml = generate_mjcf(3, 0, 0.001);
        let model = sim_mjcf::load_model(&xml).unwrap();
        let names: Vec<Option<&str>> = model.jnt_name.iter().map(|n| n.as_deref()).collect();
        assert_eq!(names, vec![Some("x0"), Some("x1"), Some("x2")]);
    }

    #[test]
    fn actuator_names() {
        let xml = generate_mjcf(3, 2, 0.001);
        let model = sim_mjcf::load_model(&xml).unwrap();
        let names: Vec<Option<&str>> = model.actuator_name.iter().map(|n| n.as_deref()).collect();
        assert_eq!(names, vec![Some("ctrl_0"), Some("ctrl_1")]);
    }

    // ── Zero-gain actuator produces zero force ──────────────────────

    #[test]
    fn zero_gain_actuator_produces_zero_force() {
        let xml = generate_mjcf(1, 1, 0.001);
        let model = sim_mjcf::load_model(&xml).unwrap();
        let mut data = model.make_data();

        // Set control to a non-zero value
        data.ctrl[0] = 5.0;

        // Step the simulation
        data.step(&model).unwrap();

        // With zero gravity, zero damping, and zero-gain actuator,
        // the particle should not have moved.
        assert_abs_diff_eq!(data.qpos[0], 0.0, epsilon = 1e-15);
        assert_abs_diff_eq!(data.qvel[0], 0.0, epsilon = 1e-15);
        assert_abs_diff_eq!(data.qfrc_actuator[0], 0.0, epsilon = 1e-15);
    }

    // ── Timestep ─────────────────────────────────────────────────────

    #[test]
    fn custom_timestep() {
        let xml = generate_mjcf(1, 0, 0.005);
        let model = sim_mjcf::load_model(&xml).unwrap();
        assert_abs_diff_eq!(model.timestep, 0.005, epsilon = 1e-15);
    }
}
