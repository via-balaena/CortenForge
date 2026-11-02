//! A geom's mass and inertia as MuJoCo 3.5.0's compiler computes them (book
//! L22; ledger L198, L367; `builder/geom.rs`'s `compute_geom_mass`,
//! `geom_volume` and `compute_geom_inertia`): the volume in MuJoCo's order of
//! operations, a plane's 0, a height field's the box of the sizes MuJoCo
//! gives it from its asset, a written mass kept only on a volume above
//! mjEPS, and a geom of mass 0 adding no inertia.
//!
//! Every document below was loaded by MuJoCo 3.5.0 on the census's unfused
//! oracle (2026-11-01); each expects MuJoCo's `body_mass`, bit for bit (a sum
//! of products and quotients of the written numbers and pi: no library call
//! to differ between platforms), and MuJoCo's `body_inertia` within a
//! relative 1e-12, exactly where it is 0.

// The values are MuJoCo's, printed to the last digit.
#![allow(clippy::unreadable_literal)]

use sim_mjcf::load_model;

/// Each document, and the `body_mass` and `body_inertia` MuJoCo 3.5.0
/// compiles for it.
type Case = (&'static str, &'static [f64], &'static [[f64; 3]]);

const CASES: &[Case] = &[
    // plane_in_static_body
    (
        r#"<mujoco><worldbody><body name="floor" pos="0 0 0"><geom type="plane" size="1 1 0.1"/></body></worldbody></mujoco>"#,
        &[0.0, 0.0],
        &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
    ),
    // plane_in_static_body_mass_written
    (
        r#"<mujoco><worldbody><body name="floor" pos="0 0 0"><geom type="plane" size="1 1 0.1" mass="5"/></body></worldbody></mujoco>"#,
        &[0.0, 0.0],
        &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
    ),
    // geom_mass_neg_plane_static
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><geom type="plane" size="1 1 0.1" mass="-1"/></body></worldbody></mujoco>"#,
        &[0.0, 0.0],
        &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
    ),
    // plane_in_mocap_body
    (
        r#"<mujoco><worldbody><body name="m" mocap="true"><geom type="plane" size="1 1 0.1"/></body></worldbody></mujoco>"#,
        &[0.0, 0.0],
        &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
    ),
    // v_plane_beside_box_static
    (
        r#"<mujoco><worldbody><body name="floor" pos="0 0 0"><geom type="plane" size="1 1 0.1"/><geom type="box" size="0.1 0.2 0.3" pos="0.5 0 0"/></body></worldbody></mujoco>"#,
        &[0.0, 48.00000000000001],
        &[
            [0.0, 0.0, 0.0],
            [2.0800000000000005, 1.6000000000000003, 0.8000000000000003],
        ],
    ),
    // plane_static_settotalmass
    (
        r#"<mujoco><compiler settotalmass="10"/><worldbody><body name="floor" pos="0 0 0"><geom type="plane" size="1 1 0.1"/></body><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        &[0.0, 0.0, 10.0],
        &[
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [
                0.04000000000000001,
                0.04000000000000001,
                0.04000000000000001,
            ],
        ],
    ),
    // val_l367_adhesion
    (
        r#"<mujoco><default><adhesion gain="0.5"/></default><worldbody><body name="floor" pos="0 0 0"><geom name="g_floor" type="plane" size="5 5 0.1"/></body><body name="b1" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1" mass="1.0"/></body></worldbody><actuator><adhesion body="b1" ctrlrange="0 1"/></actuator></mujoco>"#,
        &[0.0, 0.0, 1.0],
        &[
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [
                0.004000000000000001,
                0.004000000000000001,
                0.004000000000000001,
            ],
        ],
    ),
    // hfield_moving_mass
    (
        r#"<mujoco><asset><hfield name="h" size="1 1 1 0.5" nrow="2" ncol="2" elevation="0 0.5 1 0.25"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="hfield" hfield="h"/></body></worldbody></mujoco>"#,
        &[0.0, 4000.0],
        &[
            [0.0, 0.0, 0.0],
            [1666.6666666666667, 1666.6666666666667, 2666.6666666666665],
        ],
    ),
    // hfield_static_mass
    (
        r#"<mujoco><asset><hfield name="h" size="1 1 1 0.5" nrow="2" ncol="2" elevation="0 0.5 1 0.25"/></asset><worldbody><body name="b" pos="0 0 1"><geom type="hfield" hfield="h"/></body></worldbody></mujoco>"#,
        &[0.0, 4000.0],
        &[
            [0.0, 0.0, 0.0],
            [1666.6666666666667, 1666.6666666666667, 2666.6666666666665],
        ],
    ),
    // v_hfield_s3
    (
        r#"<mujoco><asset><hfield name="h" size="2 3 0.4 0.2" nrow="2" ncol="2" elevation="0 0.5 1 0.25"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="hfield" hfield="h"/></body></worldbody></mujoco>"#,
        &[0.0, 9600.000000000002],
        &[
            [0.0, 0.0, 0.0],
            [28928.000000000004, 12928.000000000002, 41600.00000000001],
        ],
    ),
    // v_hfield_density
    (
        r#"<mujoco><asset><hfield name="h" size="0.5 0.25 0.3 0.1" nrow="2" ncol="2" elevation="0 0.5 1 0.25"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="hfield" hfield="h" density="7"/></body></worldbody></mujoco>"#,
        &[0.0, 0.875],
        &[
            [0.0, 0.0, 0.0],
            [
                0.022786458333333332,
                0.07747395833333333,
                0.09114583333333333,
            ],
        ],
    ),
    // v_tiny_sphere_mass_static
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><geom type="sphere" size="1e-6" mass="1"/></body></worldbody></mujoco>"#,
        &[0.0, 0.0],
        &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
    ),
    // v_tiny_box_mass_static
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><geom type="box" size="1e-5 1e-5 1e-5" mass="2"/></body></worldbody></mujoco>"#,
        &[0.0, 0.0],
        &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
    ),
    // v_shell_tiny_mass
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><geom type="sphere" size="1e-8" mass="2" shellinertia="true"/></body></worldbody></mujoco>"#,
        &[0.0, 0.0],
        &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
    ),
    // val_capsule
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="capsule" size="0.2 0.5"/></body></worldbody></mujoco>"#,
        &[0.0, 159.1740277818829],
        &[
            [0.0, 0.0, 0.0],
            [23.155632252059178, 23.155632252059178, 3.049439269084494],
        ],
    ),
    // v_capsule_fromto
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="capsule" fromto="0 0 0 0 0 0.6" size="0.05"/></body></worldbody></mujoco>"#,
        &[0.0, 5.23598775598299],
        &[
            [0.0, 0.0, 0.0],
            [
                0.19785488732920722,
                0.19785488732920722,
                0.006414085001079163,
            ],
        ],
    ),
    // val_ellipsoid
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="ellipsoid" size="0.04 0.03 0.02"/></body></worldbody></mujoco>"#,
        &[0.0, 0.1005309649148734],
        &[
            [0.0, 0.0, 0.0],
            [
                2.6138050877867084e-05,
                4.021238596594936e-05,
                5.026548245743669e-05,
            ],
        ],
    ),
    // val_sphere_density
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="sphere" size="0.07"/></body></worldbody></mujoco>"#,
        &[0.0, 1.4367550402417326],
        &[
            [0.0, 0.0, 0.0],
            [
                0.002816039878873797,
                0.002816039878873797,
                0.002816039878873797,
            ],
        ],
    ),
    // v_small_box_mass_static
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><geom type="box" size="2e-5 2e-5 2e-5" mass="2"/></body></worldbody></mujoco>"#,
        &[0.0, 2.0],
        &[
            [0.0, 0.0, 0.0],
            [
                5.333333333333335e-10,
                5.333333333333335e-10,
                5.333333333333335e-10,
            ],
        ],
    ),
    // v_mass_huge_static
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><geom type="sphere" size="1e120" mass="3"/></body></worldbody></mujoco>"#,
        &[0.0, 3.0],
        &[
            [0.0, 0.0, 0.0],
            [
                1.2000000000000001e+240,
                1.2000000000000001e+240,
                1.2000000000000001e+240,
            ],
        ],
    ),
    // v_mass0_density_static
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><geom type="sphere" size="0.1" mass="0" density="5"/></body></worldbody></mujoco>"#,
        &[0.0, 0.0],
        &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
    ),
    // v_density0_static
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><geom type="sphere" size="0.1" density="0"/></body></worldbody></mujoco>"#,
        &[0.0, 0.0],
        &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
    ),
    // v_density0_huge_static
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><geom type="sphere" size="1e120" density="0"/></body></worldbody></mujoco>"#,
        &[0.0, 0.0],
        &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
    ),
    // v_hfield_mass
    (
        r#"<mujoco><asset><hfield name="h" size="0.5 0.25 0.3 0.1" nrow="2" ncol="2" elevation="0 0.5 1 0.25"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="hfield" hfield="h" mass="3"/></body></worldbody></mujoco>"#,
        &[0.0, 3.0],
        &[[0.0, 0.0, 0.0], [0.078125, 0.265625, 0.3125]],
    ),
    // plane_in_world
    (
        r#"<mujoco><worldbody><geom type="plane" size="1 1 0.1"/><body name="b" pos="0 0 1"><joint name="j" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#,
        &[0.0, 4.188790204786391],
        &[
            [0.0, 0.0, 0.0],
            [
                0.01675516081914557,
                0.01675516081914557,
                0.01675516081914557,
            ],
        ],
    ),
    // val_sphere
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="sphere" size="0.05" mass="0.8"/></body></worldbody></mujoco>"#,
        &[0.0, 0.8],
        &[
            [0.0, 0.0, 0.0],
            [
                0.0008000000000000001,
                0.0008000000000000001,
                0.0008000000000000001,
            ],
        ],
    ),
    // v_sphere_mass_odd
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="sphere" size="0.13" mass="0.7"/></body></worldbody></mujoco>"#,
        &[0.0, 0.7],
        &[[0.0, 0.0, 0.0], [0.004732, 0.004732, 0.004732]],
    ),
    // val_box
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="box" size="0.1 0.2 0.3" mass="2"/></body></worldbody></mujoco>"#,
        &[0.0, 2.0],
        &[
            [0.0, 0.0, 0.0],
            [
                0.08666666666666667,
                0.06666666666666667,
                0.03333333333333334,
            ],
        ],
    ),
    // val_box_density
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="box" size="0.1 0.2 0.3"/></body></worldbody></mujoco>"#,
        &[0.0, 48.00000000000001],
        &[
            [0.0, 0.0, 0.0],
            [2.0800000000000005, 1.6000000000000003, 0.8000000000000003],
        ],
    ),
    // v_box_mass_odd
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="box" size="0.13 0.07 0.29" mass="1.3"/></body></worldbody></mujoco>"#,
        &[0.0, 1.3],
        &[
            [0.0, 0.0, 0.0],
            [
                0.038566666666666666,
                0.04376666666666667,
                0.009446666666666667,
            ],
        ],
    ),
    // val_cylinder
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="cylinder" size="0.1 0.3"/></body></worldbody></mujoco>"#,
        &[0.0, 18.84955592153876],
        &[
            [0.0, 0.0, 0.0],
            [0.6126105674500096, 0.6126105674500096, 0.0942477796076938],
        ],
    ),
    // val_cylinder_mass
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="cylinder" size="0.1 0.3" mass="1.7"/></body></worldbody></mujoco>"#,
        &[0.0, 1.7],
        &[[0.0, 0.0, 0.0], [0.05525, 0.05525, 0.0085]],
    ),
    // v_cylinder_density
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="cylinder" size="0.07 0.21" density="900"/></body></worldbody></mujoco>"#,
        &[0.0, 5.8188579129790154],
        &[
            [0.0, 0.0, 0.0],
            [0.0926653122641908, 0.0926653122641908, 0.014256201886798589],
        ],
    ),
    // v_cylinder_small
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="cylinder" size="0.01 0.01"/></body></worldbody></mujoco>"#,
        &[0.0, 0.0062831853071795875],
        &[
            [0.0, 0.0, 0.0],
            [
                3.6651914291880924e-07,
                3.6651914291880924e-07,
                3.141592653589794e-07,
            ],
        ],
    ),
    // val_capsule_mass
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="capsule" size="0.2 0.5" mass="3"/></body></worldbody></mujoco>"#,
        &[0.0, 3.0],
        &[
            [0.0, 0.0, 0.0],
            [0.436421052631579, 0.436421052631579, 0.057473684210526316],
        ],
    ),
    // v_capsule_density
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="capsule" size="0.03 0.17" density="1100"/></body></worldbody></mujoco>"#,
        &[0.0, 1.1818671562804801],
        &[
            [0.0, 0.0, 0.0],
            [
                0.01454080190687994,
                0.01454080190687994,
                0.0005206435841088221,
            ],
        ],
    ),
    // v_ellipsoid_mass
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="ellipsoid" size="0.3 0.2 0.1" mass="2.5"/></body></worldbody></mujoco>"#,
        &[0.0, 2.5],
        &[[0.0, 0.0, 0.0], [0.025000000000000005, 0.05, 0.065]],
    ),
    // v_capsule_orders
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="capsule" size="0.03 0.13"/></body></worldbody></mujoco>"#,
        &[0.0, 0.8482300164692441],
        &[
            [0.0, 0.0, 0.0],
            [
                0.006589522006831126,
                0.006589522006831126,
                0.00037152474721352893,
            ],
        ],
    ),
    // v_ellipsoid_orders
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="ellipsoid" size="0.01 0.01 0.04"/></body></worldbody></mujoco>"#,
        &[0.0, 0.016755160819145565],
        &[
            [0.0, 0.0, 0.0],
            [
                5.696754678509493e-06,
                5.696754678509493e-06,
                6.702064327658226e-07,
            ],
        ],
    ),
    // s_plane_static
    (
        r#"<mujoco><worldbody><body name="floor" pos="0 0 0"><geom type="plane" size="1 1 0.1" shellinertia="true"/></body></worldbody></mujoco>"#,
        &[0.0, 0.0],
        &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
    ),
    // s_plane_static_mass
    (
        r#"<mujoco><worldbody><body name="floor" pos="0 0 0"><geom type="plane" size="1 1 0.1" mass="2" shellinertia="true"/></body></worldbody></mujoco>"#,
        &[0.0, 0.0],
        &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
    ),
    // s_plane_class
    (
        r#"<mujoco><default><geom shellinertia="true"/></default><worldbody><body name="floor" pos="0 0 0"><geom type="plane" size="1 1 0.1"/></body></worldbody></mujoco>"#,
        &[0.0, 0.0],
        &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
    ),
    // s_hfield_density
    (
        r#"<mujoco><asset><hfield name="h" size="1 1 1 0.5" nrow="2" ncol="2" elevation="0 0.5 1 0.25"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="hfield" hfield="h" shellinertia="true"/></body></worldbody></mujoco>"#,
        &[0.0, 16000.0],
        &[
            [0.0, 0.0, 0.0],
            [10666.666666666666, 10666.666666666666, 16000.0],
        ],
    ),
    // s_hfield_mass
    (
        r#"<mujoco><asset><hfield name="h" size="1 1 1 0.5" nrow="2" ncol="2" elevation="0 0.5 1 0.25"/></asset><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="hfield" hfield="h" mass="3" shellinertia="true"/></body></worldbody></mujoco>"#,
        &[0.0, 3.0],
        &[[0.0, 0.0, 0.0], [2.0, 2.0, 3.0]],
    ),
    // s_sphere
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="sphere" size="0.13" shellinertia="true"/></body></worldbody></mujoco>"#,
        &[0.0, 212.37166338267002],
        &[
            [0.0, 0.0, 0.0],
            [2.3927207407780826, 2.3927207407780826, 2.3927207407780826],
        ],
    ),
    // s_capsule
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="capsule" size="0.03 0.17" shellinertia="true"/></body></worldbody></mujoco>"#,
        &[0.0, 75.39822368615503],
        &[
            [0.0, 0.0, 0.0],
            [1.037542389774565, 1.037542389774565, 0.06446548125166254],
        ],
    ),
    // s_cylinder
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="cylinder" size="0.07 0.21" shellinertia="true"/></body></worldbody></mujoco>"#,
        &[0.0, 215.51325603625983],
        &[
            [0.0, 0.0, 0.0],
            [4.563493196567801, 4.563493196567801, 0.9805853149649825],
        ],
    ),
    // s_box
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="box" size="0.13 0.07 0.29" shellinertia="true"/></body></worldbody></mujoco>"#,
        &[0.0, 536.8],
        &[
            [0.0, 0.0, 0.0],
            [20.991946666666664, 23.983626666666666, 6.715680000000001],
        ],
    ),
    // s_box_mass
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="box" size="0.13 0.07 0.29" mass="1.3" shellinertia="true"/></body></worldbody></mujoco>"#,
        &[0.0, 1.3],
        &[
            [0.0, 0.0, 0.0],
            [
                0.050837426726279195,
                0.05808255340288128,
                0.016263755588673625,
            ],
        ],
    ),
    // s_sphere_orders
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="sphere" size="0.01" shellinertia="true"/></body></worldbody></mujoco>"#,
        &[0.0, 1.2566370614359175],
        &[
            [0.0, 0.0, 0.0],
            [
                8.377580409572783e-05,
                8.377580409572783e-05,
                8.377580409572783e-05,
            ],
        ],
    ),
    // s_capsule_orders
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="capsule" size="0.01 0.01" shellinertia="true"/></body></worldbody></mujoco>"#,
        &[0.0, 2.513274122871835],
        &[
            [0.0, 0.0, 0.0],
            [
                0.00043982297150257113,
                0.00043982297150257113,
                0.00020943951023931956,
            ],
        ],
    ),
    // s_cylinder_orders
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="cylinder" size="0.02 0.01" shellinertia="true"/></body></worldbody></mujoco>"#,
        &[0.0, 5.02654824574367],
        &[
            [0.0, 0.0, 0.0],
            [
                0.0010890854532444618,
                0.0010890854532444618,
                0.0015079644737231008,
            ],
        ],
    ),
    // s_box_orders
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="box" size="0.01 0.02 0.04" shellinertia="true"/></body></worldbody></mujoco>"#,
        &[0.0, 11.2],
        &[
            [0.0, 0.0, 0.0],
            [0.010026666666666666, 0.00848, 0.003146666666666667],
        ],
    ),
    // v_box_at_mjeps
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><geom type="box" size="1e-14 0.125 1" mass="1"/></body></worldbody></mujoco>"#,
        &[0.0, 0.0],
        &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
    ),
    // v_sphere_r001
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="sphere" size="0.01"/></body></worldbody></mujoco>"#,
        &[0.0, 0.004188790204786391],
        &[
            [0.0, 0.0, 0.0],
            [
                1.6755160819145565e-07,
                1.6755160819145565e-07,
                1.6755160819145565e-07,
            ],
        ],
    ),
    // v_capsule_r002
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="capsule" size="0.02 0.03" mass="1"/></body></worldbody></mujoco>"#,
        &[0.0, 1.0],
        &[
            [0.0, 0.0, 0.0],
            [
                0.0007415384615384616,
                0.0007415384615384616,
                0.0001876923076923077,
            ],
        ],
    ),
    // v_box_thin_mass
    (
        r#"<mujoco><worldbody><body name="b" pos="0 0 1"><freejoint name="f"/><geom type="box" size="0.1 0.01 0.03" mass="1.3"/></body></worldbody></mujoco>"#,
        &[0.0, 1.3],
        &[
            [0.0, 0.0, 0.0],
            [
                0.00043333333333333337,
                0.004723333333333334,
                0.004376666666666668,
            ],
        ],
    ),
    // v_plane_tilted_only
    (
        r#"<mujoco><worldbody><body name="floor" pos="0 0 0" euler="0.3 0.2 0.1"><geom type="plane" size="1 1 0.1"/></body></worldbody></mujoco>"#,
        &[0.0, 0.0],
        &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
    ),
    // v_plane_mass3
    (
        r#"<mujoco><worldbody><body name="floor" pos="0 0 0"><geom type="plane" size="1 1 0.1" mass="3"/></body></worldbody></mujoco>"#,
        &[0.0, 0.0],
        &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
    ),
];

/// Each document's `body_mass` is MuJoCo's, bit for bit.
#[test]
fn a_geoms_mass_is_mujocos() {
    let mut wrong = Vec::new();
    for (doc, mujoco, _) in CASES {
        let model = match load_model(doc) {
            Ok(model) => model,
            Err(e) => {
                wrong.push(format!("refused ({e}): {doc}"));
                continue;
            }
        };
        let got: Vec<u64> = model.body_mass.iter().map(|m| m.to_bits()).collect();
        let want: Vec<u64> = mujoco.iter().map(|m| m.to_bits()).collect();
        if got != want {
            wrong.push(format!(
                "{:?} where MuJoCo's is {mujoco:?}: {doc}",
                model.body_mass
            ));
        }
    }
    assert!(
        wrong.is_empty(),
        "{} of {}:\n{}",
        wrong.len(),
        CASES.len(),
        wrong.join("\n")
    );
}

/// Each document's `body_inertia` is MuJoCo's within a relative 1e-12, and
/// exactly 0 where MuJoCo's is (a plane, or a geom of mass 0); a value that
/// is not a number fails.
#[test]
fn a_geoms_inertia_is_mujocos() {
    let mut wrong = Vec::new();
    for (doc, _, mujoco) in CASES {
        let model = match load_model(doc) {
            Ok(model) => model,
            Err(e) => {
                wrong.push(format!("refused ({e}): {doc}"));
                continue;
            }
        };
        let got: Vec<f64> = model
            .body_inertia
            .iter()
            .flat_map(|i| i.iter().copied())
            .collect();
        let want: Vec<f64> = mujoco.iter().flatten().copied().collect();
        let close = |&(g, w): &(f64, f64)| {
            if w == 0.0 {
                g.to_bits() == w.to_bits()
            } else {
                (g - w).abs() <= 1e-12 * w.abs()
            }
        };
        if got.len() != want.len() || !got.iter().copied().zip(want).all(|p| close(&p)) {
            wrong.push(format!(
                "{:?} where MuJoCo's is {mujoco:?}: {doc}",
                model.body_inertia
            ));
        }
    }
    assert!(
        wrong.is_empty(),
        "{} of {}:\n{}",
        wrong.len(),
        CASES.len(),
        wrong.join("\n")
    );
}
