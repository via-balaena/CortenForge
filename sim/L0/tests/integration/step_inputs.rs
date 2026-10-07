//! The check every `Result` entry point runs before any work: a timestep that is not positive
//! and finite, and a `Data` made by a model of other dimensions, are refused. MuJoCo 3.5.0 checks
//! neither: it steps at any timestep, and it never reads the model signature it stores in
//! `mjData` (divergences D-STEP-TIMESTEP and D-DATA-SHAPE).

use nalgebra::{DVector, UnitQuaternion, Vector3};
use sim_core::{BodyWrench, Data, MjStage, Model, StepError};

type EntryPoint = fn(&mut Data, &Model) -> Result<(), StepError>;

/// The five `Result` entry points on `Data`.
const ENTRY_POINTS: [(&str, EntryPoint); 5] = [
    ("step", |d, m| d.step(m)),
    ("step1", |d, m| d.step1(m)),
    ("step2", |d, m| d.step2(m)),
    ("forward", |d, m| d.forward(m)),
    ("forward_skip", |d, m| {
        d.forward_skip(m, MjStage::None, false)
    }),
];

fn pendulum(links: usize) -> Model {
    Model::n_link_pendulum(links, 1.0, 1.0)
}

/// Two models with the same nq, nv, na, nu and nbody that differ in ngeom alone (0 and 1).
fn same_joints_one_more_geom() -> (Model, Model) {
    let without = pendulum(2);
    let mut with = pendulum(2);
    with.add_ground_plane();
    (without, with)
}

#[test]
fn forward_refuses_a_timestep_that_is_not_positive_and_finite() {
    for timestep in [0.0, -1e-3, f64::NAN, f64::INFINITY] {
        let mut model = pendulum(2);
        model.timestep = timestep;
        for (name, entry) in ENTRY_POINTS {
            let mut data = model.make_data();
            assert_eq!(
                entry(&mut data, &model),
                Err(StepError::InvalidTimestep),
                "{name} at timestep {timestep}"
            );
        }
    }
}

#[test]
fn step2_at_negative_timestep_does_not_run_time_backwards() {
    let mut model = pendulum(1);
    model.timestep = -1e-3;
    let mut data = model.make_data();
    data.qvel[0] = 1.0;
    assert!(data.step2(&model).is_err());
    assert_eq!(data.time, 0.0);
    assert_eq!(data.qpos[0], 0.0);
}

/// A `Data` made by one model and used with another of other dimensions. Before the check,
/// `step` panicked on the ngeom case one way and stepped the other, and panicked on the nq
/// cases both ways.
#[test]
fn step_refuses_data_made_by_another_model() {
    let (without, with) = same_joints_one_more_geom();
    let (two, three) = (pendulum(2), pendulum(3));
    for (made_by, stepped_with, case) in [
        (&without, &with, "ngeom 0, stepped with ngeom 1"),
        (&with, &without, "ngeom 1, stepped with ngeom 0"),
        (&two, &three, "nq 2, stepped with nq 3"),
        (&three, &two, "nq 3, stepped with nq 2"),
    ] {
        for (name, entry) in ENTRY_POINTS {
            let mut data = made_by.make_data();
            assert!(entry(&mut data, stepped_with).is_err(), "{name}: {case}");
        }
    }
}

#[test]
fn step_names_the_mismatched_field() {
    let (without, with) = same_joints_one_more_geom();
    let mut data = without.make_data();
    let refused = data.step(&with);
    assert_eq!(
        refused,
        Err(StepError::DataShapeMismatch {
            field: "geom_xpos",
            expected: 1,
            actual: 0,
        })
    );
    assert_eq!(
        refused.unwrap_err().to_string(),
        "data.geom_xpos has length 0, but the model needs 1: the Data was made by another \
         model, or the array was resized"
    );
}

#[test]
fn invalid_timestep_says_what_it_is() {
    assert_eq!(
        StepError::InvalidTimestep.to_string(),
        "timestep must be positive and finite"
    );
}

/// Each array the shape check compares, grown by one element in turn: the check names it, with
/// the model's length and the grown one.
#[test]
fn every_checked_array_is_named() {
    let grow: [(&str, fn(&mut Data)); 24] = [
        ("qpos", |d| d.qpos = DVector::zeros(d.qpos.len() + 1)),
        ("qvel", |d| d.qvel = DVector::zeros(d.qvel.len() + 1)),
        ("act", |d| d.act = DVector::zeros(d.act.len() + 1)),
        ("ctrl", |d| d.ctrl = DVector::zeros(d.ctrl.len() + 1)),
        ("qfrc_applied", |d| {
            d.qfrc_applied = DVector::zeros(d.qfrc_applied.len() + 1);
        }),
        ("xfrc_applied", |d| {
            d.xfrc_applied.push(BodyWrench::default())
        }),
        ("mocap_pos", |d| d.mocap_pos.push(Vector3::zeros())),
        ("mocap_quat", |d| {
            d.mocap_quat.push(UnitQuaternion::identity())
        }),
        ("xpos", |d| d.xpos.push(Vector3::zeros())),
        ("xanchor", |d| d.xanchor.push(Vector3::zeros())),
        ("geom_xpos", |d| d.geom_xpos.push(Vector3::zeros())),
        ("site_xpos", |d| d.site_xpos.push(Vector3::zeros())),
        ("ten_length", |d| d.ten_length.push(0.0)),
        ("wrap_xpos", |d| d.wrap_xpos.push(Vector3::zeros())),
        ("eq_violation", |d| d.eq_violation.push(0.0)),
        ("flexvert_xpos", |d| d.flexvert_xpos.push(Vector3::zeros())),
        ("flexedge_length", |d| d.flexedge_length.push(0.0)),
        ("flexedge_J", |d| d.flexedge_J.push(0.0)),
        ("qLD_data", |d| d.qLD_data.push(0.0)),
        ("sensordata", |d| {
            d.sensordata = DVector::zeros(d.sensordata.len() + 1);
        }),
        ("history", |d| d.history.push(0.0)),
        ("tree_asleep", |d| d.tree_asleep.push(0)),
        ("plugin_state", |d| d.plugin_state.push(0.0)),
        ("plugin_data", |d| d.plugin_data.push(None)),
    ];
    let model = pendulum(2);
    for (name, grow_one) in grow {
        let mut data = model.make_data();
        grow_one(&mut data);
        match data.forward(&model) {
            Err(StepError::DataShapeMismatch {
                field,
                expected,
                actual,
            }) => {
                assert_eq!(field, name);
                assert_eq!(actual, expected + 1, "{name}");
            }
            other => panic!("{name} grown by one: {other:?}"),
        }
    }
}
