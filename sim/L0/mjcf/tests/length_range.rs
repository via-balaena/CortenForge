//! Actuator length ranges as MuJoCo 3.5.0 makes them (`mjCModel::LengthRange`,
//! `mj_setLengthRange`): each actuator's length range computed as MuJoCo's
//! compiler computes it after `mj_setConst`, a written one kept (book L44;
//! ledgers L37, L74, L116, L272); `<compiler><lengthrange>`'s options read
//! (book L49).
//!
//! Every expected value was printed by MuJoCo 3.5.0 on the census's unfused
//! oracle (2026-11-26).

// The values are MuJoCo's, printed to the last digit.
#![allow(clippy::unwrap_used, clippy::unreadable_literal)]

use sim_mjcf::load_model;

/// What MuJoCo 3.5.0 makes of a document: each actuator's length range and
/// `acc0`; with every control at 0.5, after `mj_forward`, each actuator's
/// length and force; and the position, velocity and activation after twenty
/// steps.
struct Want {
    lengthrange: &'static [f64],
    acc0: &'static [f64],
    length: &'static [f64],
    force: &'static [f64],
    qpos: &'static [f64],
    qvel: &'static [f64],
    act: &'static [f64],
}

/// What a document should do.
enum Expect {
    /// Load as MuJoCo 3.5.0's model, and step as it does.
    Loads(Want),
    /// Be refused, as MuJoCo 3.5.0 refuses it, in words holding these:
    /// MuJoCo's, or this crate's own for the same fault.
    Refused(&'static str),
}

const CASES: &[(&str, &str, Expect)] = &[
    // a muscle on a hinge limited to ±1°: the run lands past each soft limit
    (
        "m01_hinge_1deg",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-1 1\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.018023230873209384, 0.018023230873209384],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.0010167864354604442],
            qvel: &[-0.05166121350725006],
            act: &[0.4928973829054284],
        }),
    ),
    // a muscle on a hinge limited to -30°..60°
    (
        "m02_hinge_m30_60",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.524168713951565, 1.0477674895498639],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.00876866855232844],
            qvel: &[-0.6702499477254533],
            act: &[0.4928973829054284],
        }),
    ),
    // the same beside a floor: contacts are off in the run
    (
        "m03_floor_contact",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><geom type=\"plane\" size=\"1 1 0.1\"/><body name=\"arm\" pos=\"0 0 0.05\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.524168713951565, 1.0477674895498639],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.00876866855232844],
            qvel: &[-0.6702499477254533],
            act: &[0.4928973829054284],
        }),
    ),
    // the same with stiffness, damping, friction loss and armature: springs, dampers and friction loss are off in the run
    (
        "m04_passive",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\" stiffness=\"5\" damping=\"1\" frictionloss=\"0.1\" armature=\"0.01\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.524168713951565, 1.0477674895498639],
            acc0: &[23.58343333171486],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.01198652610018191],
            qvel: &[-0.6686477540142962],
            act: &[0.4928973829054284],
        }),
    ),
    // the same with constraints and limits disabled: the run replaces the model's disable flags
    (
        "m05_flags_off",
        "<mujoco model=\"m\"><option timestep=\"0.002\"><flag constraint=\"disable\" limit=\"disable\"/></option><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.524168713951565, 1.0477674895498639],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.00876866855232844],
            qvel: &[-0.6702499477254533],
            act: &[0.4928973829054284],
        }),
    ),
    // the same under RK4: the run's step is Euler's, as mj_step2's
    (
        "m06_rk4",
        "<mujoco model=\"m\"><option timestep=\"0.002\" integrator=\"RK4\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.524168713951565, 1.0477674895498639],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.007150287697113497],
            qvel: &[-0.623295983768808],
            act: &[0.48866625576033923],
        }),
    ),
    // the same under implicit
    (
        "m07_implicit",
        "<mujoco model=\"m\"><option timestep=\"0.002\" integrator=\"implicit\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.524168713951565, 1.0477674895498639],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.008399230439169432],
            qvel: &[-0.6492329487091806],
            act: &[0.4928973829054284],
        }),
    ),
    // the same under implicitfast
    (
        "m08_implicitfast",
        "<mujoco model=\"m\"><option timestep=\"0.002\" integrator=\"implicitfast\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.524168713951565, 1.0477674895498639],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.008399230439169434],
            qvel: &[-0.6492329487091807],
            act: &[0.4928973829054284],
        }),
    ),
    // the same with sleep enabled: sleep is off in the run
    (
        "m09_sleep",
        "<mujoco model=\"m\"><option timestep=\"0.002\"><flag sleep=\"enable\"/></option><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.524168713951565, 1.0477674895498639],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.00876866855232844],
            qvel: &[-0.6702499477254533],
            act: &[0.4928973829054284],
        }),
    ),
    // the same at a model timestep of 0.0005: the run's is 0.01
    (
        "m10_timestep_small",
        "<mujoco model=\"m\"><option timestep=\"0.0005\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.524168713951565, 1.0477674895498639],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[0.0008361179654714309],
            qvel: &[0.06928646128576721],
            act: &[0.3601241495279469],
        }),
    ),
    // a gear of 2: the length is the gear times the joint's
    (
        "m11_gear2",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\" gear=\"2\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-1.04833742790313, 2.0955349790997277],
            acc0: &[61.7233523041872],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.00876866855232844],
            qvel: &[-0.6702499477254533],
            act: &[0.4928973829054284],
        }),
    ),
    // a gear of -1.5
    (
        "m12_gear_neg",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\" gear=\"-1.5\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-1.5716512343247957, 0.7862530709273474],
            acc0: &[46.292514228140405],
            length: &[-0.0],
            force: &[0.0],
            qpos: &[0.07700424943995884],
            qvel: &[3.605816478063828],
            act: &[0.4928973829054284],
        }),
    ),
    // a hinge limited to 10°..80°, its qpos0 outside the range
    (
        "m13_qpos0_outside",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"10 80\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[0.17396298684616684, 1.3968333399487298],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[0.11730331322768692],
            qvel: &[3.2396691972575247],
            act: &[0.4928973829054284],
        }),
    ),
    // a hinge with a ref
    (
        "m14_ref",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\" ref=\"20\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.524168713951565, 1.0477674895498639],
            acc0: &[30.8616761520936],
            length: &[0.3490658503988659],
            force: &[0.0],
            qpos: &[0.3346122048364484],
            qvel: &[-0.9384221413244156],
            act: &[0.4928973829054284],
        }),
    ),
    // a muscle on a limited slide
    (
        "m15_slide",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"slide\" axis=\"1 0 0\" limited=\"true\" range=\"-0.1 0.25\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.1005699383532661, 0.25056993835326613],
            acc0: &[1.0],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.02486515608696837],
            qvel: &[-1.0436632054798873],
            act: &[0.4928973829054284],
        }),
    ),
    // a muscle on a limited ball joint
    (
        "m20_ball_limited",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"ball\" limited=\"true\" range=\"0 45\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.7856751786240783, 0.7856751786240783],
            acc0: &[2275.7697456492638],
            length: &[0.0],
            force: &[0.0],
            qpos: &[
                0.9995785780117403,
                -0.0218846398028213,
                0.019067772178213878,
                0.0003859856045972702,
            ],
            qvel: &[-2.170162929601753, 1.8149682453726408, 0.07096517581550378],
            act: &[0.4928973829054284],
        }),
    ),
    // a hinge limited in radians
    (
        "m21_radian",
        "<mujoco model=\"m\"><compiler angle=\"radian\"/><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-0.5 1\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.5005699383532661, 1.0005699383532662],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.00871890638690633],
            qvel: &[-0.6640041329924139],
            act: &[0.4928973829054284],
        }),
    ),
    // a muscle's written range kept: it increases
    (
        "e01_muscle_kept",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\" lengthrange=\"-0.2 0.7\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.2, 0.7],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.004323890372645482],
            qvel: &[-0.4178841232653401],
            act: &[0.4928973829054284],
        }),
    ),
    // a motor's written range kept
    (
        "e02_motor_kept",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><motor joint=\"h\" lengthrange=\"0.2 0.7\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[0.2, 0.7],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.5],
            qpos: &[0.05110549001205765],
            qvel: &[2.4333232668807234],
            act: &[],
        }),
    ),
    // a motor's backwards written range kept: a motor's is not computed
    (
        "e03_motor_backwards_kept",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><motor joint=\"h\" lengthrange=\"0.7 0.2\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[0.7, 0.2],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.5],
            qpos: &[0.05110549001205765],
            qvel: &[2.4333232668807234],
            act: &[],
        }),
    ),
    // a muscle's backwards written range computed again
    (
        "e04_muscle_backwards_simulated",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\" lengthrange=\"0.7 0.2\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.524168713951565, 1.0477674895498639],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.00876866855232844],
            qvel: &[-0.6702499477254533],
            act: &[0.4928973829054284],
        }),
    ),
    // a muscle's empty written range computed again
    (
        "e05_muscle_equal_simulated",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\" lengthrange=\"0.5 0.5\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.524168713951565, 1.0477674895498639],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.00876866855232844],
            qvel: &[-0.6702499477254533],
            act: &[0.4928973829054284],
        }),
    ),
    // a muscle on an unlimited hinge, its range written (ledger L272)
    (
        "e07_l272_unlimited_written",
        "<mujoco model=\"m\"><worldbody><body name=\"b\" pos=\"0 0 1\"><joint name=\"h\" axis=\"0 1 0\"/><geom type=\"box\" size=\"0.15 0.03 0.03\" pos=\"0.15 0 0\"/></body></worldbody><actuator><muscle joint=\"h\" force=\"20\" lengthrange=\"-1.5 1.5\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-1.5, 1.5],
            acc0: &[30.55861141669723],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.0909536928807234],
            qvel: &[-4.4967538455284854],
            act: &[0.4928973829054284],
        }),
    ),
    // a <general> muscle's written range kept
    (
        "e08_general_written",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><general joint=\"h\" gaintype=\"muscle\" biastype=\"muscle\" dyntype=\"muscle\" ctrllimited=\"true\" ctrlrange=\"0 1\" gainprm=\"0.75 1.05 -1 200 0.5 1.6 1.5 1.3 1.2\" biasprm=\"0.75 1.05 -1 200 0.5 1.6 1.5 1.3 1.2\" dynprm=\"0.01 0.04\" lengthrange=\"-0.3 0.9\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.3, 0.9],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.005537671764054173],
            qvel: &[-0.4964406020399705],
            act: &[0.4928973829054284],
        }),
    ),
    // a motor on a limited hinge: no range computed
    (
        "n01_motor_limited",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><motor joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[0.0, 0.0],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.5],
            qpos: &[0.05110549001205765],
            qvel: &[2.4333232668807234],
            act: &[],
        }),
    ),
    // a position actuator on a limited hinge
    (
        "n02_position_limited",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><position joint=\"h\" kp=\"10\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[0.0, 0.0],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[5.0],
            qpos: &[0.16064007619666723],
            qvel: &[7.344579279062819],
            act: &[],
        }),
    ),
    // a motor on an unlimited fixed tendon
    (
        "n03_motor_fixed_tendon_unlimited",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><tendon><fixed name=\"t\"><joint joint=\"h\" coef=\"1\"/></fixed></tendon><actuator><motor tendon=\"t\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[0.0, 0.0],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.5],
            qpos: &[0.05110549001205765],
            qvel: &[2.4333232668807234],
            act: &[],
        }),
    ),
    // a velocity actuator on a limited hinge
    (
        "n04_velocity_limited",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><velocity joint=\"h\" kv=\"1\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[0.0, 0.0],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.5],
            qpos: &[0.035681491345509],
            qvel: &[1.41996571696826],
            act: &[],
        }),
    ),
    // a muscle on a limited fixed tendon
    (
        "t01_fixed_limited",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><tendon><fixed name=\"t\" limited=\"true\" range=\"-0.3 0.5\"><joint joint=\"h\" coef=\"1\"/></fixed></tendon><actuator><muscle tendon=\"t\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.3005699383532661, 0.5005699383532661],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.008920018234021975],
            qvel: &[-0.6035988536607677],
            act: &[0.4928973829054284],
        }),
    ),
    // a muscle on an unlimited fixed tendon over a limited hinge
    (
        "t02_fixed_over_limited",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><tendon><fixed name=\"t\"><joint joint=\"h\" coef=\"1\"/></fixed></tendon><actuator><muscle tendon=\"t\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.524168713951565, 1.0477674895498639],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.00876866855232844],
            qvel: &[-0.6702499477254533],
            act: &[0.4928973829054284],
        }),
    ),
    // a muscle on a limited spatial tendon
    (
        "t03_spatial_limited",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><site name=\"s0\" pos=\"0.3 0 1.5\"/><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/><site name=\"s1\" pos=\"0.3 0 0\"/></body></worldbody><tendon><spatial name=\"t\" limited=\"true\" range=\"0.3 0.6\"><site site=\"s0\"/><site site=\"s1\"/></spatial></tendon><actuator><muscle tendon=\"t\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[0.352162884370682, 0.6002583186173585],
            acc0: &[9.25850284562808],
            length: &[0.5],
            force: &[0.0],
            qpos: &[-0.013093371432172294],
            qvel: &[-0.7822337439373435],
            act: &[0.4928973829054284],
        }),
    ),
    // a muscle on an unlimited spatial tendon over a limited hinge
    (
        "t04_spatial_unlimited_joint_limited",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><site name=\"s0\" pos=\"0.3 0 1.5\"/><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/><site name=\"s1\" pos=\"0.3 0 0\"/></body></worldbody><tendon><spatial name=\"t\"><site site=\"s0\"/><site site=\"s1\"/></spatial></tendon><actuator><muscle tendon=\"t\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[0.352162884370682, 0.7745850043063421],
            acc0: &[9.25850284562808],
            length: &[0.5],
            force: &[0.0],
            qpos: &[-0.0091793850208956],
            qvel: &[-0.6799586673042997],
            act: &[0.4928973829054284],
        }),
    ),
    // a muscle on a spatial tendon wrapping a cylinder
    (
        "t05_spatial_wrap",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><site name=\"s0\" pos=\"0.3 0 1.5\"/><body pos=\"0.15 0 1.25\"><geom name=\"cyl\" type=\"cylinder\" size=\"0.04 0.1\" quat=\"0.7071068 0.7071068 0 0\" contype=\"0\" conaffinity=\"0\"/><site name=\"side\" pos=\"0 0 0.1\"/></body><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/><site name=\"s1\" pos=\"0.3 0 0\"/></body></worldbody><tendon><spatial name=\"t\"><site site=\"s0\"/><geom geom=\"cyl\" sidesite=\"side\"/><site site=\"s1\"/></spatial></tendon><actuator><muscle tendon=\"t\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[0.5029924018382411, 0.8273639165369033],
            acc0: &[9.25850284562808],
            length: &[0.5],
            force: &[0.0],
            qpos: &[-0.001615714807666414],
            qvel: &[-0.3197197653881009],
            act: &[0.4928973829054284],
        }),
    ),
    // a <general> muscle through jointinparent
    (
        "g01_jointinparent",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><general jointinparent=\"h\" gaintype=\"muscle\" biastype=\"muscle\" dyntype=\"muscle\" ctrllimited=\"true\" ctrlrange=\"0 1\" gainprm=\"0.75 1.05 -1 200 0.5 1.6 1.5 1.3 1.2\" biasprm=\"0.75 1.05 -1 200 0.5 1.6 1.5 1.3 1.2\" dynprm=\"0.01 0.04\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.524168713951565, 1.0477674895498639],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.00876866855232844],
            qvel: &[-0.6702499477254533],
            act: &[0.4928973829054284],
        }),
    ),
    // a <general> muscle through a site and a refsite
    (
        "g02_site_refsite",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><site name=\"r\" pos=\"0 0 1\"/><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"slide\" axis=\"1 0 0\" limited=\"true\" range=\"-0.1 0.2\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/><site name=\"s1\" pos=\"0.3 0 0\"/></body></worldbody><actuator><general site=\"s1\" refsite=\"r\" gear=\"1 0 0 0 0 0\" gaintype=\"muscle\" biastype=\"muscle\" dyntype=\"muscle\" ctrllimited=\"true\" ctrlrange=\"0 1\" gainprm=\"0.75 1.05 -1 200 0.5 1.6 1.5 1.3 1.2\" biasprm=\"0.75 1.05 -1 200 0.5 1.6 1.5 1.3 1.2\" dynprm=\"0.01 0.04\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[0.19943006164673388, 0.5005699383532661],
            acc0: &[1.0],
            length: &[0.3],
            force: &[0.0],
            qpos: &[-0.02377063256850973],
            qvel: &[-0.9676624780602432],
            act: &[0.4928973829054284],
        }),
    ),
    // a <general> muscle through a slider-crank
    (
        "g03_slider_crank",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"sl\" pos=\"0 0 1\"><joint name=\"sj\" type=\"slide\" axis=\"1 0 0\" limited=\"true\" range=\"-0.2 0.2\"/><geom type=\"box\" size=\"0.02 0.02 0.02\" mass=\"0.2\"/><site name=\"ss\" pos=\"0 0 0\"/></body><body name=\"arm\" pos=\"0.5 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/><site name=\"cs\" pos=\"0.1 0 0\"/></body></worldbody><actuator><general cranksite=\"cs\" slidersite=\"ss\" cranklength=\"0.45\" gaintype=\"muscle\" biastype=\"muscle\" dyntype=\"muscle\" ctrllimited=\"true\" ctrlrange=\"0 1\" gainprm=\"0.75 1.05 -1 200 0.5 1.6 1.5 1.3 1.2\" biasprm=\"0.75 1.05 -1 200 0.5 1.6 1.5 1.3 1.2\" dynprm=\"0.01 0.04\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.08663102322902505, 0.050049349985836766],
            acc0: &[3.0861676152093587],
            length: &[0.0],
            force: &[0.0],
            qpos: &[0.0, 0.07474000807654294],
            qvel: &[0.0, 3.4634477676401456],
            act: &[0.4928973829054284],
        }),
    ),
    // a <general> actuator whose gain alone is a muscle's: MuJoCo's filter takes it
    (
        "g05_general_gain_only",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><general joint=\"h\" gaintype=\"muscle\" dyntype=\"muscle\" ctrllimited=\"true\" ctrlrange=\"0 1\" gainprm=\"0.75 1.05 -1 200 0.5 1.6 1.5 1.3 1.2\" biasprm=\"0.75 1.05 -1 200 0.5 1.6 1.5 1.3 1.2\" dynprm=\"0.01 0.04\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.524168713951565, 1.0477674895498639],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.00876866855232844],
            qvel: &[-0.6702499477254533],
            act: &[0.4928973829054284],
        }),
    ),
    // two muscles on a two-link chain
    (
        "x01_two_muscles_chain",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"a\" pos=\"0 0 1\"><joint name=\"h1\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/><body pos=\"0.3 0 0\"><joint name=\"h2\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-45 45\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></body></worldbody><actuator><muscle joint=\"h1\"/><muscle joint=\"h2\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[
                -0.5240181032373927,
                1.0475846882922915,
                -0.786146172386216,
                0.786146172386216,
            ],
            acc0: &[43.22716728355382, 132.26584802526227],
            length: &[0.0, 0.0],
            force: &[0.0, 0.0],
            qpos: &[0.026122650469811885, -0.034668379149446274],
            qvel: &[1.121962486061835, -1.4029495116706354],
            act: &[0.4928973829054284, 0.4928973829054284],
        }),
    ),
    // a muscle on a hinge a joint equality ties to a limited hinge, sleep enabled (ledger L74)
    (
        "x03_l74_joint_equality_sleep",
        "<mujoco model=\"m\"><option timestep=\"0.002\"><flag sleep=\"enable\"/></option><worldbody><body pos=\"0 0 1\"><joint name=\"j\" type=\"hinge\" axis=\"0 1 0\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body><body pos=\"1 0 1\"><joint name=\"k\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-1 1\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><equality><joint joint1=\"j\" joint2=\"k\"/></equality><actuator><muscle joint=\"j\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.01886675105394705, 0.01886675105394705],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[0.004238234526822834, 0.005762512568919921],
            qvel: &[-0.0021719516194415134, 0.02255179769673062],
            act: &[0.4928973829054284],
        }),
    ),
    // a muscle on a spatial tendon, sleep enabled (ledger L74)
    (
        "x04_l74_spatial_sleep",
        "<mujoco model=\"m\"><option timestep=\"0.002\"><flag sleep=\"enable\"/></option><worldbody><site name=\"s0\" pos=\"0.3 0 1.5\"/><body pos=\"0 0 1\"><joint name=\"j\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-1 1\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/><site name=\"s1\" pos=\"0.3 0 0\"/></body></worldbody><tendon><spatial name=\"t\"><site site=\"s0\"/><site site=\"s1\"/></spatial></tendon><actuator><muscle tendon=\"t\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[0.4945933258637615, 0.5054066788848908],
            acc0: &[9.25850284562808],
            length: &[0.5],
            force: &[0.0],
            qpos: &[-0.00101670995513686],
            qvel: &[-0.051658427959042554],
            act: &[0.4928973829054284],
        }),
    ),
    // a muscle on a spatial tendon to a rim on an unlimited hinge: the geometry bounds it
    (
        "x05_rim_offset_sweep",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><site name=\"s0\" pos=\"0 0 1.5\"/><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/><site name=\"s1\" pos=\"0.2 0 0.07\"/></body></worldbody><tendon><spatial name=\"t\"><site site=\"s0\"/><site site=\"s1\"/></spatial></tendon><actuator><muscle tendon=\"t\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[0.288103799045346, 0.7118959639130171],
            acc0: &[6.507658910551009],
            length: &[0.474236228055175],
            force: &[0.0],
            qpos: &[-0.01231018322884678],
            qvel: &[-0.8620568561653865],
            act: &[0.4928973829054284],
        }),
    ),
    // a motor and a muscle on one hinge: the motor's not computed
    (
        "x07_muscle_and_motor",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><motor joint=\"h\"/><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[0.0, 0.0, -0.524168713951565, 1.0477674895498639],
            acc0: &[30.8616761520936, 30.8616761520936],
            length: &[0.0, 0.0],
            force: &[0.5, 0.0],
            qpos: &[0.0016680904820904154],
            qvel: &[-0.23466708170974118],
            act: &[0.4928973829054284],
        }),
    ),
    // a muscle on a fixed tendon a tendon equality ties to one over a slide with a ref: each tendon's length at qpos0 made first (ledger L116)
    (
        "x09_l116_limited_ref",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body pos=\"0 0 1\"><joint name=\"s1\" type=\"slide\" axis=\"1 0 0\" limited=\"true\" range=\"-0.5 0.5\" ref=\"0.3\"/><geom type=\"box\" size=\"0.05 0.05 0.05\" mass=\"1\"/></body><body pos=\"0 1 1\"><joint name=\"s2\" type=\"slide\" axis=\"1 0 0\"/><geom type=\"box\" size=\"0.05 0.05 0.05\" mass=\"1\"/></body></worldbody><tendon><fixed name=\"t1\"><joint joint=\"s1\" coef=\"1\"/></fixed><fixed name=\"t2\"><joint joint=\"s2\" coef=\"1\"/></fixed></tendon><equality><tendon tendon1=\"t1\" tendon2=\"t2\"/></equality><actuator><muscle tendon=\"t2\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.8014134585340036, 0.20141345853400375],
            acc0: &[1.0],
            length: &[0.0],
            force: &[0.0],
            qpos: &[0.27622653926305296, -0.024912987076565238],
            qvel: &[-1.2193331779934056, -1.230253733777672],
            act: &[0.4928973829054284],
        }),
    ),
    // mode all: a motor's range computed
    (
        "k01_mode_all_motor",
        "<mujoco model=\"m\"><compiler><lengthrange mode=\"all\"/></compiler><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><motor joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.524168713951565, 1.0477674895498639],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.5],
            qpos: &[0.05110549001205765],
            qvel: &[2.4333232668807234],
            act: &[],
        }),
    ),
    // mode none: a muscle's range left (0, 0)
    (
        "k02_mode_none_muscle",
        "<mujoco model=\"m\"><compiler><lengthrange mode=\"none\"/></compiler><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[0.0, 0.0],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[0.00018165182583122294],
            qvel: &[0.09082591291561147],
            act: &[0.0],
        }),
    ),
    // uselimit: a limited hinge's range copied as written at gear 2
    (
        "k03_uselimit_gear2",
        "<mujoco model=\"m\"><compiler><lengthrange uselimit=\"true\"/></compiler><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\" gear=\"2\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.5235987755982988, 1.0471975511965976],
            acc0: &[61.7233523041872],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.007777167837424033],
            qvel: &[-0.55293162474164],
            act: &[0.4928973829054284],
        }),
    ),
    // a shorter run and measured stretch
    (
        "k04_inttotal_interval",
        "<mujoco model=\"m\"><compiler><lengthrange inttotal=\"0.5\" interval=\"0.1\"/></compiler><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.5244349583492071, 1.0527977532533574],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.008740111535007603],
            qvel: &[-0.6691663529738527],
            act: &[0.4928973829054284],
        }),
    ),
    // an unlimited hinge's run long enough to converge: the range is the options', not the model's
    (
        "k06_unlimited_inttotal22",
        "<mujoco model=\"m\"><compiler><lengthrange inttotal=\"22\"/></compiler><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-422.00283166536036, 422.00283166536036],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.015759108057461488],
            qvel: &[-1.1449794768388977],
            act: &[0.4928973829054284],
        }),
    ),
    // the same, longer
    (
        "k07_unlimited_inttotal30",
        "<mujoco model=\"m\"><compiler><lengthrange inttotal=\"30\"/></compiler><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-582.8041649908383, 582.8041649908383],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.015760522064177603],
            qvel: &[-1.145140077582],
            act: &[0.4928973829054284],
        }),
    ),
    // useexisting false: a written range computed again
    (
        "k12_useexisting_false",
        "<mujoco model=\"m\"><compiler><lengthrange useexisting=\"false\"/></compiler><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\" lengthrange=\"-0.2 0.7\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.524168713951565, 1.0477674895498639],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.00876866855232844],
            qvel: &[-0.6702499477254533],
            act: &[0.4928973829054284],
        }),
    ),
    // a run timestep of 0: the model's
    (
        "k13_timestep_zero",
        "<mujoco model=\"m\"><compiler><lengthrange timestep=\"0\"/></compiler><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.5241687139515647, 1.0477674895498637],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.00876866855232844],
            qvel: &[-0.6702499477254533],
            act: &[0.4928973829054284],
        }),
    ),
    // a negative run timestep: the model's
    (
        "k14_timestep_negative",
        "<mujoco model=\"m\"><compiler><lengthrange timestep=\"-1\"/></compiler><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.5241687139515647, 1.0477674895498637],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.00876866855232844],
            qvel: &[-0.6702499477254533],
            act: &[0.4928973829054284],
        }),
    ),
    // mode muscleuser on a muscle
    (
        "k16_muscleuser",
        "<mujoco model=\"m\"><compiler><lengthrange mode=\"muscleuser\"/></compiler><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.524168713951565, 1.0477674895498639],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.00876866855232844],
            qvel: &[-0.6702499477254533],
            act: &[0.4928973829054284],
        }),
    ),
    // a force cap below the force the run's acceleration takes
    (
        "k17_maxforce",
        "<mujoco model=\"m\"><compiler><lengthrange maxforce=\"0.1\"/></compiler><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.5237332894837584, 1.0473320650820575],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.008764966267921837],
            qvel: &[-0.6700275945191472],
            act: &[0.4928973829054284],
        }),
    ),
    // another acceleration and velocity time constant
    (
        "k19_accel_timeconst",
        "<mujoco model=\"m\"><compiler><lengthrange accel=\"10\" timeconst=\"0.5\"/></compiler><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.5239711099064508, 1.0475698855047497],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.00876698887850384],
            qvel: &[-0.6701490658193897],
            act: &[0.4928973829054284],
        }),
    ),
    // uselimit: a limited fixed tendon's range copied as written
    (
        "k20_uselimit_tendon",
        "<mujoco model=\"m\"><compiler><lengthrange uselimit=\"true\"/></compiler><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><tendon><fixed name=\"t\" limited=\"true\" range=\"-0.3 0.5\"><joint joint=\"h\" coef=\"2\"/></fixed></tendon><actuator><muscle tendon=\"t\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.3, 0.5],
            acc0: &[61.7233523041872],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.007048317417093659],
            qvel: &[-0.42473921556983757],
            act: &[0.4928973829054284],
        }),
    ),
    // an unlimited hinge under a tolerance its run meets
    (
        "k22_tolrange",
        "<mujoco model=\"m\"><compiler><lengthrange tolrange=\"0.5\"/></compiler><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-181.0027413207507, 181.0027413207507],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.015752287331667635],
            qvel: &[-1.1442049346274399],
            act: &[0.4928973829054284],
        }),
    ),
    // a second <compiler>'s <lengthrange> sets only what it writes: the first's mode kept
    (
        "k28_two_compilers_mode_kept",
        "<mujoco model=\"m\"><compiler><lengthrange mode=\"all\"/></compiler><compiler><lengthrange inttotal=\"5\"/></compiler><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><motor joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.524168713951565, 1.0477674895498639],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.5],
            qpos: &[0.05110549001205765],
            qvel: &[2.4333232668807234],
            act: &[],
        }),
    ),
    // an empty second <lengthrange> keeps the first's options
    (
        "k29_two_compilers_second_empty",
        "<mujoco model=\"m\"><compiler><lengthrange inttotal=\"0.5\" interval=\"0.1\"/></compiler><compiler><lengthrange/></compiler><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.5244349583492071, 1.0527977532533574],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[-0.008740111535007603],
            qvel: &[-0.6691663529738527],
            act: &[0.4928973829054284],
        }),
    ),
    // a muscle on a tree that starts asleep: sleep is off in the run, so it moves
    (
        "m22_sleep_init",
        "<mujoco model=\"m\"><option timestep=\"0.002\"><flag sleep=\"enable\"/></option><worldbody><body name=\"arm\" pos=\"0 0 1\" sleep=\"init\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Loads(Want {
            lengthrange: &[-0.524168713951565, 1.0477674895498639],
            acc0: &[30.8616761520936],
            length: &[0.0],
            force: &[0.0],
            qpos: &[0.0],
            qvel: &[0.0],
            act: &[0.0],
        }),
    ),
    // a muscle on an unlimited hinge: the run's length still moves
    (
        "m16_unlimited_hinge",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Refused(
            "Lengthrange computation did not converge in actuator 0:\n  eval (-181.003, -140.808)\n  range (-181.003, 181.003)",
        ),
    ),
    // a muscle on an unlimited slide
    (
        "m17_unlimited_slide",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"slide\" axis=\"1 0 0\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Refused(
            "Lengthrange computation did not converge in actuator 0:\n  eval (-181.003, -140.808)\n  range (-181.003, 181.003)",
        ),
    ),
    // a muscle on an unlimited hinge with stiffness, which is off in the run
    (
        "m18_unlimited_stiff",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" stiffness=\"5\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Refused(
            "Lengthrange computation did not converge in actuator 0:\n  eval (-181.003, -140.808)\n  range (-181.003, 181.003)",
        ),
    ),
    // a muscle on a spatial tendon between coincident sites: an empty range
    (
        "t06_spatial_coincident",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><site name=\"s0\" pos=\"0 0 1\"/><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/><site name=\"s1\" pos=\"0 0 0\"/></body></worldbody><tendon><spatial name=\"t\"><site site=\"s0\"/><site site=\"s1\"/></spatial></tendon><actuator><muscle tendon=\"t\"/></actuator></mujoco>\n",
        Expect::Refused("Invalid lengthrange (0, 0) in actuator 0"),
    ),
    // a <general> muscle through a site without a refsite: no length
    (
        "g04_site_no_refsite",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/><site name=\"s1\" pos=\"0.3 0 0\"/></body></worldbody><actuator><general site=\"s1\" gear=\"1 0 0 0 0 0\" gaintype=\"muscle\" biastype=\"muscle\" dyntype=\"muscle\" ctrllimited=\"true\" ctrlrange=\"0 1\" gainprm=\"0.75 1.05 -1 200 0.5 1.6 1.5 1.3 1.2\" biasprm=\"0.75 1.05 -1 200 0.5 1.6 1.5 1.3 1.2\" dynprm=\"0.01 0.04\"/></actuator></mujoco>\n",
        Expect::Refused("Invalid lengthrange (0, 0) in actuator 0"),
    ),
    // a <general> muscle through a body transmission: no length
    (
        "g06_body",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/><geom type=\"box\" size=\"0.05 0.05 0.05\" pos=\"0.3 0 0\" mass=\"0.1\"/></body></worldbody><actuator><general body=\"arm\" gaintype=\"muscle\" biastype=\"muscle\" dyntype=\"muscle\" ctrllimited=\"true\" ctrlrange=\"0 1\" gainprm=\"0.75 1.05 -1 200 0.5 1.6 1.5 1.3 1.2\" biasprm=\"0.75 1.05 -1 200 0.5 1.6 1.5 1.3 1.2\" dynprm=\"0.01 0.04\"/></actuator></mujoco>\n",
        Expect::Refused("Invalid lengthrange (0, 0) in actuator 0"),
    ),
    // a muscle on a fixed tendon a tendon equality ties to one over an unlimited slide
    (
        "x02_l116_fixed_tendons_ref",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><body pos=\"0 0 1\"><joint name=\"s1\" type=\"slide\" axis=\"1 0 0\" ref=\"0.3\"/><geom type=\"box\" size=\"0.05 0.05 0.05\" mass=\"1\"/></body><body pos=\"0 1 1\"><joint name=\"s2\" type=\"slide\" axis=\"1 0 0\"/><geom type=\"box\" size=\"0.05 0.05 0.05\" mass=\"1\"/></body></worldbody><tendon><fixed name=\"t1\"><joint joint=\"s1\" coef=\"1\"/></fixed><fixed name=\"t2\"><joint joint=\"s2\" coef=\"1\"/></fixed></tendon><equality><tendon tendon1=\"t1\" tendon2=\"t2\"/></equality><actuator><muscle tendon=\"t2\"/></actuator></mujoco>\n",
        Expect::Refused(
            "Lengthrange computation did not converge in actuator 0:\n  eval (-90.5017, -70.4044)\n  range (-90.5017, 90.5017)",
        ),
    ),
    // a spatial tendon to a rim in line with its anchor at qpos0: no force moves it
    (
        "x06_rim_in_line",
        "<mujoco model=\"m\"><option timestep=\"0.002\"/><worldbody><site name=\"s0\" pos=\"0 0 1.5\"/><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/><site name=\"s1\" pos=\"0 0 0.2\"/></body></worldbody><tendon><spatial name=\"t\"><site site=\"s0\"/><site site=\"s1\"/></spatial></tendon><actuator><muscle tendon=\"t\"/></actuator></mujoco>\n",
        Expect::Refused("Invalid lengthrange (0.3, 0.3) in actuator 0"),
    ),
    // a <general> muscle on a free joint: its length is 0
    (
        "x08_free_body_muscle_gear",
        "<mujoco model=\"m\"><option timestep=\"0.002\" gravity=\"0 0 0\"/><worldbody><body name=\"f\" pos=\"0 0 1\"><freejoint name=\"fj\"/><geom type=\"box\" size=\"0.1 0.1 0.1\" mass=\"1\"/></body></worldbody><actuator><general joint=\"fj\" gear=\"1 0 0 0 0 0\" gaintype=\"muscle\" biastype=\"muscle\" dyntype=\"muscle\" ctrllimited=\"true\" ctrlrange=\"0 1\" gainprm=\"0.75 1.05 -1 200 0.5 1.6 1.5 1.3 1.2\" biasprm=\"0.75 1.05 -1 200 0.5 1.6 1.5 1.3 1.2\" dynprm=\"0.01 0.04\"/></actuator></mujoco>\n",
        Expect::Refused("Invalid lengthrange (0, 0) in actuator 0"),
    ),
    // mode all on an adhesion: no length
    (
        "k05_adhesion_mode_all",
        "<mujoco model=\"m\"><compiler><lengthrange mode=\"all\"/></compiler><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><adhesion body=\"arm\" ctrlrange=\"0 1\" gain=\"1\"/></actuator></mujoco>\n",
        Expect::Refused("Invalid lengthrange (0, 0) in actuator 0"),
    ),
    // an unlimited hinge's run too short to converge
    (
        "k08_unlimited_inttotal20",
        "<mujoco model=\"m\"><compiler><lengthrange inttotal=\"20\"/></compiler><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Refused(
            "Lengthrange computation did not converge in actuator 0:\n  eval (-381.802, -341.602)\n  range (-381.802, 381.802)",
        ),
    ),
    // a measured stretch longer than the run
    (
        "k21_interval_over_inttotal",
        "<mujoco model=\"m\"><compiler><lengthrange inttotal=\"1\" interval=\"2\"/></compiler><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Refused(
            "Lengthrange computation did not converge in actuator 0:\n  eval (-0.556633, 0)\n  range (-0.556633, 1.06951)",
        ),
    ),
    // a run timestep that is not a number
    (
        "k09_timestep_bad",
        "<mujoco model=\"m\"><compiler><lengthrange timestep=\"x\"/></compiler><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Refused("bad number \"x\" in attribute 'timestep' on <lengthrange>"),
    ),
    // an acceleration that is not a number
    (
        "k10_accel_bad",
        "<mujoco model=\"m\"><compiler><lengthrange accel=\"x\"/></compiler><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Refused("bad number \"x\" in attribute 'accel' on <lengthrange>"),
    ),
    // a flag that is not false or true
    (
        "k11_useexisting_bad",
        "<mujoco model=\"m\"><compiler><lengthrange useexisting=\"bogus\"/></compiler><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Refused("invalid keyword \"bogus\" in attribute 'useexisting' on <lengthrange>"),
    ),
    // a mode that is not MuJoCo's
    (
        "k18_mode_bad",
        "<mujoco model=\"m\"><compiler><lengthrange mode=\"bogus\"/></compiler><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Refused("invalid keyword \"bogus\" in attribute 'mode' on <lengthrange>"),
    ),
    // two numbers for one
    (
        "k23_two_values",
        "<mujoco model=\"m\"><compiler><lengthrange accel=\"10 20\"/></compiler><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\" limited=\"true\" range=\"-30 60\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Refused("expected exactly 1 values, found 2 in attribute 'accel' on <lengthrange>"),
    ),
    // a run timestep exact in binary on an unlimited hinge: a step lands on inttotal - interval, which is not measured
    (
        "k31_unlimited_timestep_quarter",
        "<mujoco model=\"m\"><compiler><lengthrange timestep=\"0.25\"/></compiler><option timestep=\"0.002\"/><worldbody><body name=\"arm\" pos=\"0 0 1\"><joint name=\"h\" type=\"hinge\" axis=\"0 1 0\"/><geom type=\"capsule\" fromto=\"0 0 0 0.3 0 0\" size=\"0.03\" mass=\"1\"/></body></worldbody><actuator><muscle joint=\"h\"/></actuator></mujoco>\n",
        Expect::Refused(
            "Lengthrange computation did not converge in actuator 0:\n  eval (-200.495, -160.943)\n  range (-200.495, 200.495)",
        ),
    ),
];

/// 0 on macOS arm64, where MuJoCo's values were printed, and 1e-9 of an
/// array's largest entry elsewhere, where the platform's libm gives other
/// last bits.
const TOLERANCE: f64 = if cfg!(all(target_os = "macos", target_arch = "aarch64")) {
    0.0
} else {
    1e-9
};

fn close(ours: &[f64], mujoco: &[f64]) -> bool {
    if ours.len() != mujoco.len() {
        return false;
    }
    // Equal bits pass; otherwise both entries are finite and within the
    // tolerance of the array's largest finite entry, and two zeros have one
    // sign.
    let scale = mujoco
        .iter()
        .filter(|v| v.is_finite())
        .fold(0.0_f64, |m, v| m.max(v.abs()));
    ours.iter().zip(mujoco).all(|(a, b)| {
        a.to_bits() == b.to_bits()
            || (a.is_finite()
                && b.is_finite()
                && (a - b).abs() <= TOLERANCE * scale
                && !(*a == 0.0 && *b == 0.0))
    })
}

/// The fields of the model `doc` loads to that are not MuJoCo 3.5.0's
/// `want`; `load`, `make_data`, `forward` or `step` where that fails, so that
/// one case's failure leaves the others compared.
fn differing(doc: &str, want: &Want) -> Vec<&'static str> {
    let Ok(m) = load_model(doc) else {
        return vec!["load"];
    };
    let Ok(mut d) = m.try_make_data() else {
        return vec!["make_data"];
    };
    d.ctrl.fill(0.5);
    if d.forward(&m).is_err() {
        return vec!["forward"];
    }
    let length = d.actuator_length.clone();
    let force = d.actuator_force.clone();
    for _ in 0..20 {
        if d.step(&m).is_err() {
            return vec!["step"];
        }
    }
    let lengthrange: Vec<f64> = m
        .actuator_lengthrange
        .iter()
        .flat_map(|&range| <[f64; 2]>::from(range))
        .collect();
    [
        ("lengthrange", close(&lengthrange, want.lengthrange)),
        ("acc0", close(&m.actuator_acc0, want.acc0)),
        ("length", close(&length, want.length)),
        ("force", close(&force, want.force)),
        ("qpos", close(d.qpos.as_slice(), want.qpos)),
        ("qvel", close(d.qvel.as_slice(), want.qvel)),
        ("act", close(d.act.as_slice(), want.act)),
    ]
    .into_iter()
    .filter(|&(_, same)| !same)
    .map(|(field, _)| field)
    .collect()
}

/// Each document MuJoCo 3.5.0 loads loads here as its model, and steps as it
/// does.
#[test]
fn each_length_range_is_mujocos() {
    let mut bad = Vec::new();
    for (name, doc, expect) in CASES {
        if let Expect::Loads(want) = expect {
            let fields = differing(doc, want);
            if !fields.is_empty() {
                bad.push(format!("{name}: {fields:?}"));
            }
        }
    }
    assert!(bad.is_empty(), "{bad:#?}");
}

/// Each document MuJoCo 3.5.0 refuses is refused here, in the words its
/// case gives.
#[test]
fn each_length_range_mujoco_refuses_is_refused() {
    let mut bad = Vec::new();
    for (name, doc, expect) in CASES {
        if let Expect::Refused(words) = expect {
            match load_model(doc) {
                Ok(_) => bad.push(format!("{name}: loads")),
                Err(e) if !e.to_string().contains(words) => bad.push(format!("{name}: {e}")),
                Err(_) => {}
            }
        }
    }
    assert!(bad.is_empty(), "{bad:#?}");
}

/// `<lengthrange>` options MuJoCo 3.5.0 never returns on (each killed at
/// 120 s on the oracle), refused here: a finite acceleration whose run resets
/// at every step as unstable (registry `D-LENGTHRANGE-RESET`), a written
/// `nan` or `inf` as not finite (`D-NONFINITE`). Made at run time, so that the
/// census, which runs each of its documents on MuJoCo, holds none of them.
#[test]
fn options_mujoco_never_returns_on_are_refused() {
    let doc = |attrs: &str| {
        format!(
            r#"<mujoco model="m"><compiler><lengthrange {attrs}/></compiler><option timestep="0.002"/><worldbody><body name="arm" pos="0 0 1"><joint name="h" type="hinge" axis="0 1 0" limited="true" range="-30 60"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"/></body></worldbody><actuator><muscle joint="h"/></actuator></mujoco>"#
        )
    };
    let unstable = load_model(&doc(r#"accel="1e12""#)).map(|_| ()).unwrap_err();
    assert_eq!(
        unstable.to_string(),
        "Unstable lengthrange simulation in actuator 0"
    );
    for attrs in [
        r#"accel="nan""#,
        r#"accel="inf""#,
        r#"accel="-inf""#,
        r#"timeconst="nan""#,
        r#"inttotal="inf""#,
    ] {
        let error = load_model(&doc(attrs)).map(|_| ()).unwrap_err();
        assert!(
            matches!(error, sim_mjcf::MjcfError::NonFinite { .. }),
            "{attrs}: {error}"
        );
    }
}
