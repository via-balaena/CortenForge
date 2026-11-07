//! A flex's `elastic2d` (book L32), against MuJoCo 3.5.0 run on the unfused
//! oracle for the same documents: a dim-2 flex bends only under `bend` or
//! `both` (its flaps, hinges and coefficients; `user_model.cc:3504-3510`,
//! `user_mesh.cc:4363-4374`), and the elasticity sim-core lacks is refused
//! where MuJoCo would compute it: a membrane (`stretch`, `both`) and a dim-3
//! flex's Young's modulus.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::fmt::Write as _;

use sim_core::FlexBendingType;
use sim_mjcf::{MjcfError, load_model};

const SLIDES: &str = r#"<joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/><joint type="slide" axis="0 0 1"/>"#;
const INERTIAL: &str = r#"<inertial pos="0 0 0" mass="0.1" diaginertia="1e-6 1e-6 1e-6"/>"#;

/// A flex of `dim` on bodies `v0`, `v1`, ... at `positions`, each with three
/// slides unless it is in `pinned`, gravity off, contacts off, with
/// `<elasticity {elasticity}/>`.
fn flex(
    dim: usize,
    positions: &[&str],
    elements: &str,
    elasticity: &str,
    pinned: &[usize],
) -> String {
    let mut bodies = String::new();
    for (i, pos) in positions.iter().enumerate() {
        let joints = if pinned.contains(&i) { "" } else { SLIDES };
        write!(
            bodies,
            r#"<body name="v{i}" pos="{pos}">{joints}{INERTIAL}</body>"#
        )
        .unwrap();
    }
    let names: Vec<String> = (0..positions.len()).map(|i| format!("v{i}")).collect();
    format!(
        r#"<mujoco><option gravity="0 0 0"/><worldbody>{bodies}</worldbody><deformable>
<flex name="f" dim="{dim}" body="{}" element="{elements}"><contact contype="0" conaffinity="0"/><elasticity {elasticity}/></flex>
</deformable></mujoco>"#,
        names.join(" ")
    )
}

const SQUARE: [&str; 4] = ["0 0 0", "1 0 0", "0 1 0", "1 1 0"];

/// The unit square of two triangles, `0 1 2  1 3 2`.
fn square(elasticity: &str) -> String {
    flex(2, &SQUARE, "0 1 2  1 3 2", elasticity, &[])
}

/// A dim-1 flex of one edge, and a dim-3 flex of one tetrahedron.
fn cable(elasticity: &str) -> String {
    flex(1, &SQUARE[..2], "0 1", elasticity, &[])
}

fn tetrahedron(elasticity: &str) -> String {
    flex(
        3,
        &["0 0 0", "1 0 0", "0 1 0", "0 0 1"],
        "0 1 2 3",
        elasticity,
        &[],
    )
}

const BENT_FLAPS: [[i32; 2]; 5] = [[0, 3], [1, -1], [2, -1], [1, -1], [2, -1]];

fn nonzero(values: &[f64]) -> usize {
    values.iter().filter(|v| **v != 0.0).count()
}

#[test]
fn a_flex_bends_only_under_bend_or_both() {
    // MuJoCo: the square's flaps, 16 nonzero bending entries and no membrane
    // under `bend`; every flap `[-1, -1]` and no bending without it.
    let model = load_model(&square(r#"young="1e4" thickness="0.01" elastic2d="bend""#)).unwrap();
    assert_eq!(model.flexedge_flap, BENT_FLAPS);
    assert_eq!(nonzero(&model.flex_bending), 16);
    assert_eq!(model.nflexhinge, 1);
    assert!(model.flex_bend_stiffness[0] > 0.0);
    for unbent in ["", r#"elastic2d="none""#] {
        let model = load_model(&square(&format!(
            r#"young="1e4" thickness="0.01" {unbent}"#
        )))
        .unwrap();
        assert_eq!(model.flexedge_flap, [[-1, -1]; 5], "{unbent}");
        assert_eq!(nonzero(&model.flex_bending), 0, "{unbent}");
        assert_eq!(model.nflexhinge, 0, "{unbent}");
        assert_eq!(
            model.flex_bend_stiffness[0].to_bits(),
            0.0_f64.to_bits(),
            "{unbent}"
        );
    }
    // MuJoCo stores the flaps without a Young's modulus too, and makes no
    // bending; `both` without one makes no membrane either.
    for bends in [r#"elastic2d="bend""#, r#"elastic2d="both""#] {
        let model = load_model(&square(bends)).unwrap();
        assert_eq!(model.flexedge_flap, BENT_FLAPS, "{bends}");
        assert_eq!(nonzero(&model.flex_bending), 0, "{bends}");
    }
}

#[test]
fn stretching_is_refused_where_mujoco_makes_a_membrane() {
    // MuJoCo computes its membrane (`Stencil2D`) for a dim-2 flex that
    // stretches with a Young's modulus and a positive thickness; sim-core has
    // none (registry `D-FLEX-MEMBRANE`).
    for stretches in ["stretch", "both"] {
        let xml = square(&format!(
            r#"young="1e4" thickness="0.01" elastic2d="{stretches}""#
        ));
        let err = load_model(&xml).unwrap_err();
        assert!(
            matches!(&err, MjcfError::Unsupported { .. }),
            "{stretches}: {err}"
        );
    }
    // Everywhere else MuJoCo makes none, and loads it: no Young's modulus, no
    // thickness, a thickness of 0, a dim-1 flex.
    for loads in [
        square(r#"elastic2d="stretch""#),
        square(r#"young="1e4" elastic2d="stretch""#),
        square(r#"young="1e4" thickness="0" elastic2d="stretch""#),
        cable(r#"young="1e4" thickness="0.01" elastic2d="stretch""#),
    ] {
        let model = load_model(&loads).unwrap();
        assert!(
            model.flexedge_flap.iter().all(|f| *f == [-1, -1]),
            "{loads}"
        );
    }
}

#[test]
fn a_dim3_flexs_youngs_modulus_is_refused() {
    // MuJoCo gives it tetrahedral elasticity (`Stencil3D`) whatever
    // `elastic2d` says; sim-core has none (registry `D-FLEX-TET-ELASTICITY`).
    for elasticity in [
        r#"young="1e4""#,
        r#"young="1e4" thickness="0.01" elastic2d="stretch""#,
    ] {
        let err = load_model(&tetrahedron(elasticity)).unwrap_err();
        assert!(
            matches!(&err, MjcfError::Unsupported { .. }),
            "{elasticity}: {err}"
        );
    }
    let model = load_model(&tetrahedron(r#"elastic2d="bend""#)).unwrap();
    assert_eq!(model.flexedge_flap, [[-1, -1]; 6]);
}

#[test]
fn bending_with_a_youngs_modulus_needs_a_thickness() {
    // MuJoCo: "thickness must be positive for bending stiffness" for `bend`
    // and `both` with a Young's modulus and no thickness; a thickness of 0
    // loads with no bending, and no Young's modulus loads.
    for bends in ["bend", "both"] {
        let err = load_model(&square(&format!(r#"young="1e4" elastic2d="{bends}""#))).unwrap_err();
        assert!(
            matches!(&err, MjcfError::InvalidValue { attribute, reason, .. }
                if attribute == "thickness" && reason == "thickness must be positive for bending stiffness"),
            "{bends}: {err}"
        );
    }
    let model = load_model(&square(r#"young="1e4" thickness="0" elastic2d="bend""#)).unwrap();
    assert_eq!(model.flexedge_flap, BENT_FLAPS);
    assert_eq!(nonzero(&model.flex_bending), 0);
}

#[test]
fn a_bending_model_needs_elastic2d_bend_or_both() {
    // This crate's `bending_model` (A9 Q7): written without bending, refused;
    // the default, not written, is never refused.
    let err = load_model(&square(
        r#"young="1e4" thickness="0.01" bending_model="bridson""#,
    ))
    .unwrap_err();
    assert!(
        matches!(&err, MjcfError::InvalidValue { attribute, .. } if attribute == "bending_model"),
        "{err}"
    );
    let model = load_model(&square(
        r#"young="1e4" thickness="0.01" elastic2d="bend" bending_model="bridson""#,
    ))
    .unwrap();
    assert_eq!(model.flex_bending_type, vec![FlexBendingType::Bridson]);
    load_model(&square(r#"young="1e4" thickness="0.01""#)).unwrap();
}

#[test]
fn elastic2d_takes_mujocos_keywords() {
    for keyword in ["none", "bend", "stretch", "both"] {
        load_model(&square(&format!(r#"elastic2d="{keyword}""#))).unwrap();
    }
    // MuJoCo: "invalid keyword: 'bent'".
    let err = load_model(&square(r#"elastic2d="bent""#)).unwrap_err();
    assert!(matches!(&err, MjcfError::InvalidKeyword { .. }), "{err}");
}

#[test]
fn the_poisson_ratio_is_checked_first() {
    // MuJoCo: "Poisson ratio must be in [0, 0.5)" before any bending check.
    let err = load_model(&square(r#"young="1e4" poisson="0.6" elastic2d="bend""#)).unwrap_err();
    assert!(
        matches!(&err, MjcfError::InvalidValue { attribute, .. } if attribute == "poisson"),
        "{err}"
    );
}

/// A `<deformable><flexcomp>` grid with `children`.
fn flexcomp(children: &str) -> String {
    format!(
        r#"<mujoco><option gravity="0 0 0"/><deformable>
<flexcomp name="c" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" dim="2"><contact contype="0" conaffinity="0"/>{children}</flexcomp>
</deformable></mujoco>"#
    )
}

#[test]
fn a_flexcomp_that_stretches_has_no_edge_equality() {
    // MuJoCo's flexcomp: "elasticity and edge constraints cannot both be
    // present" for `stretch` or `both` beside `<edge equality="true">`;
    // `bend` beside it loads with its equality.
    for stretches in ["stretch", "both"] {
        let xml = flexcomp(&format!(
            r#"<edge equality="true"/><elasticity elastic2d="{stretches}"/>"#
        ));
        let err = load_model(&xml).unwrap_err();
        assert!(
            matches!(&err, MjcfError::InvalidValue { reason, .. }
                if reason == "elasticity and edge constraints cannot both be present"),
            "{stretches}: {err}"
        );
    }
    let model = load_model(&flexcomp(
        r#"<elasticity elastic2d="bend"/><edge equality="true"/>"#,
    ))
    .unwrap();
    assert_eq!(model.neq, 1);
    load_model(&flexcomp(r#"<elasticity elastic2d="stretch"/>"#)).unwrap();
}

/// An irregular flat diamond whose shared edge (1, 2) the first triangle
/// holds as (2, 1).
const DIAMOND: [&str; 4] = ["0 0 0", "1 0.1 0", "0.2 0.9 0", "1.3 1.1 0"];

#[test]
fn a_flat_diamond_at_rest_takes_no_bending_force() {
    // A bending energy is stationary at its flat rest shape. MuJoCo computes
    // the coefficients in the first triangle's edge order and applies them in
    // the sorted one, and gives this diamond max |qfrc_spring| 1.63e-5 at rest
    // (2.2e-19 as `0 1 2  1 3 2`); this crate computes and applies in one
    // order (A9 Q2, registry `D-FLEX-BEND-ORDER`). A pin: it passes before
    // book L32 too.
    let elasticity = r#"young="1e4" poisson="0.3" thickness="0.01" elastic2d="bend""#;
    let model = load_model(&flex(2, &DIAMOND, "0 2 1  2 3 1", elasticity, &[])).unwrap();
    let mut data = model.make_data();
    data.forward(&model).unwrap();
    let biggest = data.qfrc_spring.iter().fold(0.0_f64, |m, f| m.max(f.abs()));
    assert!(biggest < 1e-12, "{biggest}");
}

#[test]
fn a_pinned_vertex_leaves_its_free_neighbours_bending_as_unpinned() {
    // MuJoCo refuses bending with a pinned vertex ("pins are not supported
    // for bending", its own TODO); this crate loads it (A9 Q3, registry
    // `D-FLEX-BEND-PINS`): the free vertices' bending forces, the square
    // folded by v3's height, equal the unpinned square's.
    let elasticity = r#"young="1e4" poisson="0.3" thickness="0.01" elastic2d="bend""#;
    let pinned = load_model(&flex(2, &SQUARE, "0 1 2  1 3 2", elasticity, &[0])).unwrap();
    let free = load_model(&square(elasticity)).unwrap();
    let mut a = pinned.make_data();
    let mut b = free.make_data();
    // v3's z: the pinned square's dof 8, the free one's 11.
    a.qpos[8] = 0.1;
    b.qpos[11] = 0.1;
    a.forward(&pinned).unwrap();
    b.forward(&free).unwrap();
    for k in 0..9 {
        let (x, y) = (a.qfrc_spring[k], b.qfrc_spring[k + 3]);
        assert!(
            (x - y).abs() <= 1e-12 * y.abs().max(1.0),
            "dof {k}: {x} vs {y}"
        );
    }
    assert!(a.qfrc_spring.iter().any(|f| *f != 0.0), "the fold bends");
}
