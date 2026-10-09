//! The hybrid transition derivative against pure finite differences, swept
//! over the inputs that decide what the forward pass acts on.
//!
//! [`mjd_transition_hybrid`] computes the velocity, activation and control
//! columns of `A` and `B` analytically and the rest by finite differences;
//! [`mjd_transition_fd`] differences the whole step. On one model and state
//! the two must agree to finite-difference accuracy, entry by entry, in `A`,
//! `B`, `C` and `D`. Two blocks:
//!
//! - **structure:** every actuator kind on every transmission, with the root
//!   body on a hinge, a slide, a ball or a free joint, under each integrator, with
//!   and without joint damping and stiffness, at a control inside its range;
//! - **inputs:** every actuator kind under each state (actuation disabled,
//!   its group disabled, its force clamped, its activation past its range)
//!   and each control (inside, at and past its range, NaN,
//!   infinite; limited and unlimited), on a hinge and on a site
//!   transmission, under each integrator, with and without damping.
//!
//! Every fixture is one MuJoCo 3.5.0 loads, with another integrator in place
//! of `implicitspringdamper` (ours). A case may differ only inside a
//! class [`KNOWN_GAPS`] lists, and every listed class must still differ in
//! some case, so the list shrinks as the gaps are fixed. A case inside a
//! listed class is not checked further.
//!
//! What it cannot see: a difference below the tolerance (a ULP-level one);
//! forward differences (`centered: false`); a tree asleep at the state, where
//! a position or velocity nudge wakes the tree (MuJoCo's `mj_wake`) and
//! neither finite-difference pass restores the sleep state between columns
//! (MuJoCo's `mjd_stepFD` restores the full physics state, the control and
//! the warm start, which hold no sleep state), so the result depends on the
//! order of the passes; and a gap the forward pass and the finite
//! differences share, since finite differences are the reference here (they
//! are compared with MuJoCo's in `derivatives.rs`).

use nalgebra::DMatrix;
use sim_core::{
    Data, DerivativeConfig, Model, TransitionMatrices, Warning, mjd_transition_fd,
    mjd_transition_hybrid,
};
use std::collections::BTreeMap;
use std::fmt::Write as _;

/// An entry differs when `|hybrid − fd| > ATOL + RTOL · max(|hybrid|, |fd|)`.
const ATOL: f64 = 1e-6;
const RTOL: f64 = 1e-5;

#[derive(Clone, Copy)]
struct Kind {
    name: &'static str,
    element: &'static str,
    attrs: &'static str,
    /// The control range MuJoCo requires, when it requires one.
    forced_range: Option<&'static str>,
    /// For a kind with an activation, the attributes that give it a range
    /// (empty when its own attributes give one); `None` for a kind with no
    /// activation, and for `<cylinder>` and `<muscle>`, which take no
    /// `actlimited`.
    act_range: Option<&'static str>,
}

const ACT_RANGE: &str = r#" actlimited="true" actrange="-1 1""#;

const MUSCLE: &str = "muscle";
const ADHESION: &str = "adhesion";

const KINDS: &[Kind] = &[
    Kind {
        name: "motor",
        element: "motor",
        attrs: "",
        forced_range: None,
        act_range: None,
    },
    Kind {
        name: "position",
        element: "position",
        attrs: r#"kp="5""#,
        forced_range: None,
        act_range: None,
    },
    Kind {
        name: "velocity",
        element: "velocity",
        attrs: r#"kv="2""#,
        forced_range: None,
        act_range: None,
    },
    Kind {
        name: "damper",
        element: "damper",
        attrs: r#"kv="2""#,
        forced_range: Some("0 1"),
        act_range: None,
    },
    // `<intvelocity kp="5" actrange="-1 1">` as MuJoCo expands it; sim-mjcf
    // does not read the element yet.
    Kind {
        name: "intvelocity",
        element: "general",
        attrs: r#"dyntype="integrator" gainprm="5" biastype="affine" biasprm="0 -5 0" actlimited="true" actrange="-1 1""#,
        forced_range: None,
        act_range: Some(""),
    },
    Kind {
        name: "cylinder",
        element: "cylinder",
        attrs: r#"timeconst="0.05" area="0.5""#,
        forced_range: None,
        act_range: None,
    },
    Kind {
        name: "filterexact affine",
        element: "general",
        attrs: r#"dyntype="filterexact" dynprm="0.05" gaintype="affine" gainprm="1 0.5 -0.3" biastype="affine" biasprm="0 -1 -0.5""#,
        forced_range: None,
        act_range: Some(ACT_RANGE),
    },
    Kind {
        name: "integrator",
        element: "general",
        attrs: r#"dyntype="integrator" gainprm="2""#,
        forced_range: None,
        act_range: Some(ACT_RANGE),
    },
    Kind {
        name: "actearly filter",
        element: "general",
        attrs: r#"dyntype="filter" dynprm="0.05" gaintype="affine" gainprm="1 2 0" actearly="true""#,
        forced_range: None,
        act_range: Some(ACT_RANGE),
    },
    Kind {
        name: MUSCLE,
        element: "muscle",
        attrs: "",
        forced_range: Some("0 1"),
        act_range: None,
    },
    Kind {
        name: ADHESION,
        element: "adhesion",
        attrs: r#"gain="2""#,
        forced_range: Some("0 1"),
        act_range: None,
    },
];

#[derive(Clone, Copy, PartialEq, Eq)]
enum Root {
    Hinge,
    Slide,
    Ball,
    Free,
}

impl Root {
    const ALL: [Self; 4] = [Self::Hinge, Self::Slide, Self::Ball, Self::Free];

    fn name(self) -> &'static str {
        match self {
            Self::Hinge => "hinge root",
            Self::Slide => "slide root",
            Self::Ball => "ball root",
            Self::Free => "free root",
        }
    }

    fn joint(self, springs: &str) -> String {
        match self {
            Self::Hinge => {
                format!(r#"<joint name="j1" type="hinge" axis="0 1 0" range="-2 2"{springs}/>"#)
            }
            Self::Slide => {
                format!(r#"<joint name="j1" type="slide" axis="0 0 1" range="-1 1"{springs}/>"#)
            }
            Self::Ball => format!(r#"<joint name="j1" type="ball"{springs}/>"#),
            Self::Free => format!(r#"<joint name="j1" type="free"{springs}/>"#),
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Trn {
    JointRoot,
    JointInParentRoot,
    JointChild,
    FixedTendon,
    SpatialTendon,
    Site,
    SiteRef,
    SliderCrank,
    Body,
}

impl Trn {
    const ALL: [Self; 9] = [
        Self::JointRoot,
        Self::JointInParentRoot,
        Self::JointChild,
        Self::FixedTendon,
        Self::SpatialTendon,
        Self::Site,
        Self::SiteRef,
        Self::SliderCrank,
        Self::Body,
    ];

    fn name(self) -> &'static str {
        match self {
            Self::JointRoot => "joint j1",
            Self::JointInParentRoot => "jointinparent j1",
            Self::JointChild => "joint j2",
            Self::FixedTendon => "fixed tendon",
            Self::SpatialTendon => "spatial tendon",
            Self::Site => "site",
            Self::SiteRef => "site + refsite",
            Self::SliderCrank => "slider-crank",
            Self::Body => "body",
        }
    }

    /// A ball or free joint takes a scalar gear: the forward pass applies
    /// `gear[0]` to the joint's first dof only, where MuJoCo applies the
    /// whole gear.
    fn attrs(self) -> &'static str {
        match self {
            Self::JointRoot => r#"joint="j1" gear="1.5""#,
            Self::JointInParentRoot => r#"jointinparent="j1" gear="1.5""#,
            Self::JointChild => r#"joint="j2" gear="1.5""#,
            Self::FixedTendon => r#"tendon="tf" gear="1.5""#,
            Self::SpatialTendon => r#"tendon="ts" gear="1.5""#,
            Self::Site => r#"site="tip" gear="0 0 1 0 0 0""#,
            Self::SiteRef => r#"site="tip" refsite="w" gear="1 0 0 0 0 0.5""#,
            Self::SliderCrank => {
                r#"cranksite="crank" slidersite="slider" cranklength="0.4" gear="1.5""#
            }
            Self::Body => r#"body="b3""#,
        }
    }
}

/// Whether MuJoCo 3.5.0 loads this actuator: adhesion acts only through a
/// body and only adhesion does; MuJoCo's schema refuses `site` on a
/// `<muscle>`, its length range does not converge for this muscle on a ball
/// joint, a spatial tendon or a slider-crank, and on a free joint it is
/// (0, 0), which MuJoCo refuses.
fn mujoco_loads(kind: &Kind, trn: Trn, root: Root) -> bool {
    if (kind.name == ADHESION) != (trn == Trn::Body) {
        return false;
    }
    if kind.name == MUSCLE {
        let on_ball_or_free_root = matches!(root, Root::Ball | Root::Free)
            && matches!(trn, Trn::JointRoot | Trn::JointInParentRoot);
        let refused = matches!(
            trn,
            Trn::Site | Trn::SiteRef | Trn::SliderCrank | Trn::SpatialTendon
        );
        return !on_ball_or_free_root && !refused;
    }
    true
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Plain,
    ActuationOff,
    GroupOff,
    ForceClamped,
    ActPastRange,
}

impl State {
    const ALL: [Self; 5] = [
        Self::Plain,
        Self::ActuationOff,
        Self::GroupOff,
        Self::ForceClamped,
        Self::ActPastRange,
    ];

    fn name(self) -> &'static str {
        match self {
            Self::Plain => "plain",
            Self::ActuationOff => "actuation disabled",
            Self::GroupOff => "group disabled",
            Self::ForceClamped => "force clamped",
            Self::ActPastRange => "act past its range",
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
enum Ctrl {
    Inside,
    AtUpper,
    Past,
    Nan,
    Inf,
}

impl Ctrl {
    const ALL: [Self; 5] = [
        Self::Inside,
        Self::AtUpper,
        Self::Past,
        Self::Nan,
        Self::Inf,
    ];

    fn name(self) -> &'static str {
        match self {
            Self::Inside => "ctrl inside",
            Self::AtUpper => "ctrl at upper",
            Self::Past => "ctrl past",
            Self::Nan => "ctrl NaN",
            Self::Inf => "ctrl inf",
        }
    }

    fn value(self) -> f64 {
        match self {
            Self::Inside => 0.3,
            Self::AtUpper => 1.0,
            Self::Past => 2.0,
            Self::Nan => f64::NAN,
            Self::Inf => f64::INFINITY,
        }
    }
}

const EULER: &str = "Euler";
const IMPLICITFAST: &str = "implicitfast";
const IMPLICIT: &str = "implicit";
const ISD: &str = "implicitspringdamper";
const INTEGRATORS: [&str; 4] = [EULER, IMPLICITFAST, IMPLICIT, ISD];

#[derive(Clone, Copy)]
struct Case {
    kind: Kind,
    trn: Trn,
    root: Root,
    state: State,
    ctrl: Ctrl,
    limited: bool,
    integrator: &'static str,
    damped: bool,
}

impl Case {
    fn label(&self) -> String {
        format!(
            "{} | {} | {} | {} | {} {} | {} | {}",
            self.kind.name,
            self.trn.name(),
            self.root.name(),
            self.state.name(),
            self.ctrl.name(),
            if self.limited { "limited" } else { "unlimited" },
            self.integrator,
            if self.damped { "damped" } else { "undamped" },
        )
    }

    /// The axis values the report counts coverage by.
    fn axes(&self) -> [String; 6] {
        [
            format!("kind {}", self.kind.name),
            format!("trn {}", self.trn.name()),
            self.root.name().to_string(),
            format!("state {}", self.state.name()),
            self.ctrl.name().to_string(),
            self.integrator.to_string(),
        ]
    }

    fn xml(&self) -> String {
        let springs = if self.damped {
            r#" damping="0.3" stiffness="2""#
        } else {
            ""
        };
        let group_option = if self.state == State::GroupOff {
            r#" actuatorgroupdisable="2""#
        } else {
            ""
        };
        let flags = if self.state == State::ActuationOff {
            r#"<flag actuation="disable"/>"#
        } else {
            ""
        };
        let mut actuator = format!(
            r#"<{} name="a" {} {}"#,
            self.kind.element,
            self.trn.attrs(),
            self.kind.attrs
        );
        if let Some(range) = self.kind.forced_range {
            let _ = write!(actuator, r#" ctrlrange="{range}""#);
        } else if self.limited {
            actuator.push_str(r#" ctrlrange="-1 1""#);
        }
        match self.state {
            State::GroupOff => actuator.push_str(r#" group="2""#),
            State::ForceClamped => {
                actuator.push_str(r#" forcelimited="true" forcerange="-0.01 0.01""#);
            }
            State::ActPastRange => actuator.push_str(self.kind.act_range.unwrap_or_default()),
            _ => {}
        }
        actuator.push_str("/>");
        let root_joint = self.root.joint(springs);
        let integrator = self.integrator;
        format!(
            r#"<mujoco>
  <compiler angle="radian"/>
  <option timestep="0.01" integrator="{integrator}"{group_option}>{flags}</option>
  <default><geom contype="0" conaffinity="0"/></default>
  <worldbody>
    <site name="w" pos="0.3 0.2 0.5"/>
    <site name="slider" pos="0.3 0 0.6" zaxis="1 0 0"/>
    <body name="b1" pos="0 0 1">
      {root_joint}
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="1"/>
      <body name="b2" pos="0.3 0 0">
        <joint name="j2" type="hinge" axis="0 1 0" range="-2 2"{springs}/>
        <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="0.7"/>
        <site name="tip" pos="0.3 0 0"/>
        <site name="crank" pos="0.15 0 0"/>
        <body name="b3" pos="0.3 0 0">
          <joint name="j3" type="slide" axis="1 0 0" range="-1 1"{springs}/>
          <geom type="sphere" size="0.05" mass="0.5"/>
          <site name="s3" pos="0 0 0.05"/>
        </body>
      </body>
    </body>
  </worldbody>
  <tendon>
    <fixed name="tf" range="-3 3"><joint joint="j2" coef="1"/><joint joint="j3" coef="-0.5"/></fixed>
    <spatial name="ts" range="0 3"><site site="w"/><site site="s3"/></spatial>
  </tendon>
  <actuator>
    {actuator}
    <motor name="m2" joint="j3" gear="1"/>
  </actuator>
  <sensor>
    <actuatorfrc actuator="a"/>
    <jointvel joint="j2"/>
    <framepos objtype="site" objname="tip"/>
    <accelerometer site="tip"/>
  </sensor>
</mujoco>"#
        )
    }
}

/// A class of cases where the hybrid is known to differ from finite
/// differences, with the cases it covers.
struct Gap {
    name: &'static str,
    covers: fn(&Case) -> bool,
}

const KNOWN_GAPS: &[Gap] = &[
    // The hybrid spreads the gear over every dof of the joint; the forward
    // pass applies it to the first.
    Gap {
        name: "joint transmission on a ball or free joint",
        covers: |c| {
            matches!(c.root, Root::Ball | Root::Free)
                && matches!(c.trn, Trn::JointRoot | Trn::JointInParentRoot)
        },
    },
];

/// What one case gave.
enum Outcome {
    /// The loader or `try_make_data` refused the model, or both derivatives
    /// returned an error.
    Refused,
    /// The step from the state finds a bad position, velocity or
    /// acceleration and resets, so the derivative there is not one of the
    /// dynamics.
    StepWarns,
    /// The two ran. `differs` holds the first entry outside the tolerance, or
    /// the error only one of them returned.
    Ran {
        differs: Option<String>,
        /// The largest entry difference, when they agree.
        worst: f64,
        /// No constraint row, no bad or clamped control, and the hybrid did
        /// not return pure finite differences' own matrices (it does for a
        /// model it routes to them whole): the hybrid's analytic columns ran.
        analytic: bool,
        /// The force sat at its range, for the force-clamped state (true for
        /// the other states).
        held: bool,
    },
}

fn compare(name: &str, hybrid: &DMatrix<f64>, fd: &DMatrix<f64>) -> Result<f64, String> {
    if hybrid.shape() != fd.shape() {
        return Err(format!(
            "{name} shape {:?} vs {:?}",
            hybrid.shape(),
            fd.shape()
        ));
    }
    let mut worst = 0.0_f64;
    for r in 0..fd.nrows() {
        for c in 0..fd.ncols() {
            let (h, f) = (hybrid[(r, c)], fd[(r, c)]);
            if (h.is_nan() && f.is_nan()) || (h.is_infinite() && h == f) {
                continue;
            }
            let diff = (h - f).abs();
            let tol = ATOL + RTOL * h.abs().max(f.abs());
            if !(h.is_finite() && f.is_finite()) || diff > tol {
                return Err(format!("{name}[{r},{c}] hybrid {h:e}, fd {f:e}"));
            }
            worst = worst.max(diff);
        }
    }
    Ok(worst)
}

fn compare_all(hybrid: &TransitionMatrices, fd: &TransitionMatrices) -> Result<f64, String> {
    let mut worst = compare("A", &hybrid.A, &fd.A)?.max(compare("B", &hybrid.B, &fd.B)?);
    match (&hybrid.C, &fd.C) {
        (Some(h), Some(f)) => worst = worst.max(compare("C", h, f)?),
        (None, None) => {}
        _ => return Err("C present on one side only".into()),
    }
    match (&hybrid.D, &fd.D) {
        (Some(h), Some(f)) => worst = worst.max(compare("D", h, f)?),
        (None, None) => {}
        _ => return Err("D present on one side only".into()),
    }
    Ok(worst)
}

fn same_bits(a: &DMatrix<f64>, b: &DMatrix<f64>) -> bool {
    a.shape() == b.shape()
        && a.iter()
            .zip(b.iter())
            .all(|(x, y)| x.to_bits() == y.to_bits())
}

/// The case's model and its state after a forward pass, or `None` when the
/// loader, `try_make_data` or the forward pass refuses it.
fn nominal(case: &Case) -> Option<(Model, Data)> {
    let model = sim_mjcf::load_model(&case.xml()).ok()?;
    let mut data = model.try_make_data().ok()?;
    let (q1, v1) = (model.jnt_qpos_adr[0], model.jnt_dof_adr[0]);
    match case.root {
        Root::Hinge => {
            data.qpos[q1] = 0.4;
            data.qvel[v1] = 0.7;
        }
        Root::Slide => {
            data.qpos[q1] = 0.1;
            data.qvel[v1] = 0.7;
        }
        Root::Ball | Root::Free => {
            // A free joint's position first, then for both 0.3 rad about
            // (1, 1, 0)/√2.
            let (q, v) = if case.root == Root::Free {
                data.qpos[q1] = 0.05;
                data.qpos[q1 + 1] = -0.02;
                data.qpos[q1 + 2] = 1.0;
                data.qvel[v1] = 0.2;
                data.qvel[v1 + 1] = -0.1;
                data.qvel[v1 + 2] = 0.3;
                (q1 + 3, v1 + 3)
            } else {
                (q1, v1)
            };
            let (s, c) = (0.15_f64.sin(), 0.15_f64.cos());
            let k = s / 2.0_f64.sqrt();
            data.qpos[q] = c;
            data.qpos[q + 1] = k;
            data.qpos[q + 2] = k;
            data.qpos[q + 3] = 0.0;
            data.qvel[v] = 0.7;
            data.qvel[v + 1] = -0.4;
            data.qvel[v + 2] = 0.3;
        }
    }
    data.qpos[model.jnt_qpos_adr[1]] = 0.5;
    data.qvel[model.jnt_dof_adr[1]] = 1.0;
    data.qpos[model.jnt_qpos_adr[2]] = 0.1;
    data.qvel[model.jnt_dof_adr[2]] = -0.5;
    let act = if case.state == State::ActPastRange {
        1.5
    } else {
        0.4
    };
    data.act.fill(act);
    data.ctrl[0] = case.ctrl.value();
    data.ctrl[1] = 0.2;
    data.forward(&model).ok()?;
    Some((model, data))
}

fn run(case: &Case) -> Outcome {
    let Some((model, data)) = nominal(case) else {
        return Outcome::Refused;
    };
    let mut probe = data.clone();
    if probe.step(&model).is_err()
        || [Warning::BadQpos, Warning::BadQvel, Warning::BadQacc]
            .iter()
            .any(|&w| probe.warnings[w as usize].count > data.warnings[w as usize].count)
    {
        return Outcome::StepWarns;
    }
    let ctrl = data.ctrl[0];
    let held = case.state != State::ForceClamped
        || !ctrl.is_finite()
        || data.actuator_force[0].abs() == 0.01;
    let config = DerivativeConfig {
        eps: 1e-6,
        centered: true,
        use_analytical: true,
        compute_sensor_derivatives: true,
    };
    let hybrid = mjd_transition_hybrid(&model, &data, &config);
    let fd = mjd_transition_fd(&model, &data, &config);
    let (lo, hi) = model.actuator_ctrlrange[0];
    let mut analytic = data.efc_type.is_empty() && ctrl.is_finite() && (lo..=hi).contains(&ctrl);
    let (differs, worst) = match (hybrid, fd) {
        (Ok(h), Ok(f)) => {
            analytic &= !same_bits(&h.A, &f.A) || !same_bits(&h.B, &f.B);
            match compare_all(&h, &f) {
                Ok(worst) => (None, worst),
                Err(e) => (Some(e), 0.0),
            }
        }
        (Err(_), Err(_)) => return Outcome::Refused,
        (h, f) => (Some(format!("hybrid {:?}, fd {:?}", h.err(), f.err())), 0.0),
    };
    Outcome::Ran {
        differs,
        worst,
        analytic,
        held,
    }
}

fn structure_cases() -> Vec<Case> {
    let mut cases = Vec::new();
    for kind in KINDS {
        for trn in Trn::ALL {
            for root in Root::ALL {
                if !mujoco_loads(kind, trn, root) {
                    continue;
                }
                for integrator in INTEGRATORS {
                    for damped in [false, true] {
                        cases.push(Case {
                            kind: *kind,
                            trn,
                            root,
                            state: State::Plain,
                            ctrl: Ctrl::Inside,
                            limited: true,
                            integrator,
                            damped,
                        });
                    }
                }
            }
        }
    }
    cases
}

fn input_cases() -> Vec<Case> {
    let mut cases = Vec::new();
    for kind in KINDS {
        for trn in [Trn::JointChild, Trn::SiteRef, Trn::Body] {
            if !mujoco_loads(kind, trn, Root::Hinge) {
                continue;
            }
            for state in State::ALL {
                if state == State::ActPastRange && kind.act_range.is_none() {
                    continue;
                }
                for ctrl in Ctrl::ALL {
                    for limited in [true, false] {
                        if !limited && (kind.forced_range.is_some() || ctrl == Ctrl::AtUpper) {
                            continue;
                        }
                        for integrator in INTEGRATORS {
                            for damped in [false, true] {
                                cases.push(Case {
                                    kind: *kind,
                                    trn,
                                    root: Root::Hinge,
                                    state,
                                    ctrl,
                                    limited,
                                    integrator,
                                    damped,
                                });
                            }
                        }
                    }
                }
            }
        }
    }
    cases
}

/// Runs every case, prints the report, and returns how many cases each known
/// gap covered that differed. Panics if a case outside the known gaps
/// differs, was refused, steps from its state into a reset, or did not hold
/// its state.
fn sweep(block: &str, cases: &[Case]) -> Vec<usize> {
    let mut failures = Vec::new();
    let mut known = vec![0_usize; KNOWN_GAPS.len()];
    let mut agree = 0_usize;
    let mut analytic = 0_usize;
    let mut worst = 0.0_f64;
    // Per axis value: cases that ran, of them through the analytic columns.
    let mut coverage: BTreeMap<String, [usize; 2]> = BTreeMap::new();
    for case in cases {
        let axes = case.axes();
        match run(case) {
            Outcome::Refused => failures.push(format!("REFUSED {}", case.label())),
            Outcome::StepWarns => failures.push(format!("STEP WARNS {}", case.label())),
            Outcome::Ran {
                differs,
                worst: w,
                analytic: a,
                held,
            } => {
                for axis in &axes {
                    let entry = coverage.entry(axis.clone()).or_default();
                    entry[0] += 1;
                    entry[1] += usize::from(a);
                }
                if !held {
                    failures.push(format!("STATE DID NOT HOLD {}", case.label()));
                }
                match differs {
                    None => {
                        agree += 1;
                        analytic += usize::from(a);
                        worst = worst.max(w);
                    }
                    Some(e) => match KNOWN_GAPS.iter().position(|gap| (gap.covers)(case)) {
                        Some(g) => known[g] += 1,
                        None => failures.push(format!("DIFFERS {}: {e}", case.label())),
                    },
                }
            }
        }
    }
    let mut report = format!(
        "{block}: {} cases, {agree} agree ({analytic} through the analytic columns), \
         {} inside known gaps, {} failures; largest agreeing difference {worst:e}\n",
        cases.len(),
        known.iter().sum::<usize>(),
        failures.len(),
    );
    for (axis, [ran, through]) in &coverage {
        let _ = writeln!(report, "  {axis}: {ran} ran, {through} analytic");
    }
    for (gap, n) in KNOWN_GAPS.iter().zip(&known) {
        let _ = writeln!(report, "  known gap, {n} differ: {}", gap.name);
    }
    for line in &failures {
        let _ = writeln!(report, "  {line}");
    }
    eprintln!("{report}");
    assert!(failures.is_empty(), "{report}");
    known
}

#[test]
fn hybrid_matches_finite_differences_across_actuators() {
    let structure = sweep("structure", &structure_cases());
    let inputs = sweep("inputs", &input_cases());
    let stale: Vec<&str> = KNOWN_GAPS
        .iter()
        .zip(structure.iter().zip(&inputs))
        .filter(|(_, (s, i))| **s + **i == 0)
        .map(|(gap, _)| gap.name)
        .collect();
    assert!(
        stale.is_empty(),
        "known gaps no case differs in any more (delete them): {stale:?}"
    );
}
