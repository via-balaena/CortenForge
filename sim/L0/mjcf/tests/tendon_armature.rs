//! A tendon's armature as MuJoCo 3.5.0 reads and applies it (ledger L164).
//!
//! MuJoCo adds `armature · Jᵀ·J` to the mass matrix on the matrix's pattern
//! after `mj_crb` (`mj_tendonArmature`, `engine_core_smooth.c:1745-1808`), so
//! a term between dofs of two trees is dropped, and `armature · (J̇ · qvel)`
//! times `J` to the bias force after `mj_rne` (`mj_tendonBias`, `:2687`),
//! which the inverse dynamics' sum carries; both skip a sleeping tendon. Its
//! compiler makes every body that a tendon with armature passes over through
//! a site or a wrapped geom not simple (`FinalizeSimple`,
//! `user_model.cc:3922-3939`; a fixed tendon's joints name no body), refuses
//! a spatial tendon's negative armature (`user_objects.cc:6342`) and armature
//! on a tendon that wraps a geom (`:6389`).
//!
//! Every document and value below was measured on the census's unfused
//! oracle (2026-10-31): the matrices and forces after `mj_forward` and then
//! `mj_inverse`, and in the sleep cases after a reset and each step. They are
//! compared bit for bit: the arithmetic is MuJoCo's, in its order, a square
//! root its only library call.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
// The values are bit patterns, written as MuJoCo's values' raw bits.
#![allow(clippy::unreadable_literal)]

use sim_mjcf::{MjcfError, load_model};

/// One state of a case, every value as its bits.
struct State {
    qpos: &'static [u64],
    qvel: &'static [u64],
    /// The mass matrix, row by row.
    qm: &'static [u64],
    qfrc_bias: &'static [u64],
    qacc: &'static [u64],
    qfrc_inverse: &'static [u64],
}

struct Case {
    name: &'static str,
    xml: &'static str,
    body_simple: &'static [u8],
    dof_simplenum: &'static [usize],
    states: &'static [State],
}

const CASES: &[Case] = &[
    Case {
        name: "fixed armature",
        xml: r#"<mujoco><option timestep="0.002"/><worldbody><site name="s0" pos="0.1 0 1.2"/><body name="b1" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s1" pos="0.05 0 -0.1"/><body name="b2" pos="0 0 -0.3"><joint name="j2" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s2" pos="0.05 0 -0.2"/></body></body><body name="c" pos="0.5 0 1"><joint name="k" type="slide" axis="1 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.002 0.002 0.002"/><site name="s3" pos="0 0 0"/></body></worldbody><tendon><fixed name="t" armature="0.7"><joint joint="j1" coef="1"/><joint joint="j2" coef="-0.5"/><joint joint="k" coef="2"/></fixed></tendon></mujoco>"#,
        body_simple: &[1, 0, 0, 2],
        dof_simplenum: &[0, 0, 1],
        states: &[
            State {
                qpos: &[0x3fd3333333333333, 0xbfd999999999999a, 0x3fb999999999999a],
                qvel: &[0x3fe999999999999a, 0xbfe0000000000000, 0x3fd3333333333333],
                qm: &[
                    0x3fea246f98c50507,
                    0xbfd419010b120536,
                    0x0000000000000000,
                    0xbfd419010b120536,
                    0x3fc85a1cac083126,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x400e666666666666,
                ],
                qfrc_bias: &[0x3fe25e55ba7978c5, 0xbfb43d47b5f452a6, 0x0000000000000000],
                qacc: &[0xbff7c38f0220d9e0, 0xc00049b02c2830c1, 0x0000000000000000],
                qfrc_inverse: &[0x3ca0000000000000, 0xbc80000000000000, 0x0000000000000000],
            },
            State {
                qpos: &[0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
                qvel: &[0x3ff3333333333333, 0x3fe6666666666666, 0xbfd999999999999a],
                qm: &[
                    0x3fea4189374bc6a8,
                    0xbfd3fbe76c8b4395,
                    0x0000000000000000,
                    0xbfd3fbe76c8b4395,
                    0x3fc85a1cac083127,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x400e666666666666,
                ],
                qfrc_bias: &[0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
                qacc: &[0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
                qfrc_inverse: &[0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
            },
            State {
                qpos: &[0xbfe3333333333333, 0x3feccccccccccccd, 0xbfc999999999999a],
                qvel: &[0xbfd3333333333333, 0x3ff8000000000000, 0x3feccccccccccccd],
                qm: &[
                    0x3fe9b60bda3b5882,
                    0xbfd48764c99bb1ba,
                    0x0000000000000000,
                    0xbfd48764c99bb1ba,
                    0x3fc85a1cac083126,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x400e666666666666,
                ],
                qfrc_bias: &[0xbff0d7be3dab439a, 0x3fcc08b0e7087177, 0x0000000000000000],
                qacc: &[0x4004d0a0fe542a9d, 0x4009e27b0733b7fa, 0x0000000000000000],
                qfrc_inverse: &[0x0000000000000000, 0x3c98000000000000, 0x0000000000000000],
            },
        ],
    },
    Case {
        name: "spatial armature",
        xml: r#"<mujoco><option timestep="0.002"/><worldbody><site name="s0" pos="0.1 0 1.2"/><body name="b1" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s1" pos="0.05 0 -0.1"/><body name="b2" pos="0 0 -0.3"><joint name="j2" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s2" pos="0.05 0 -0.2"/></body></body><body name="c" pos="0.5 0 1"><joint name="k" type="slide" axis="1 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.002 0.002 0.002"/><site name="s3" pos="0 0 0"/></body></worldbody><tendon><spatial name="t" armature="0.3"><site site="s0"/><site site="s1"/><site site="s2"/></spatial></tendon></mujoco>"#,
        body_simple: &[0, 0, 0, 2],
        dof_simplenum: &[0, 0, 1],
        states: &[
            State {
                qpos: &[0x3fd3333333333333, 0xbfd999999999999a, 0x3fb999999999999a],
                qvel: &[0x3fe999999999999a, 0xbfe0000000000000, 0x3fd3333333333333],
                qm: &[
                    0x3fbe19657c09b34a,
                    0x3fa30ab869a58435,
                    0x0000000000000000,
                    0x3fa30ab869a58435,
                    0x3f9208f49195f0d5,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3ff0000000000000,
                ],
                qfrc_bias: &[0x3fe256b75072b028, 0xbfb4b38f9e01ff28, 0x0000000000000000],
                qacc: &[0xc0330dae1137e617, 0x404669b2fee7f8b0, 0x0000000000000000],
                qfrc_inverse: &[0x3cb0000000000000, 0x3cb0000000000000, 0x0000000000000000],
            },
            State {
                qpos: &[0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
                qvel: &[0x3ff3333333333333, 0x3fe6666666666666, 0xbfd999999999999a],
                qm: &[
                    0x3fbf2e1b90a2119c,
                    0x3fa3d541b222bae7,
                    0x0000000000000000,
                    0x3fa3d541b222bae7,
                    0x3f90624dd2f1a9fd,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3ff0000000000000,
                ],
                qfrc_bias: &[0xbf66b498eaadda92, 0xbf614394c6a43765, 0x0000000000000000],
                qacc: &[0xbfb54b9229492a51, 0x3fd5519871501d09, 0x0000000000000000],
                qfrc_inverse: &[0x3c30000000000000, 0x0000000000000000, 0x0000000000000000],
            },
            State {
                qpos: &[0xbfe3333333333333, 0x3feccccccccccccd, 0xbfc999999999999a],
                qvel: &[0xbfd3333333333333, 0x3ff8000000000000, 0x3feccccccccccccd],
                qm: &[
                    0x3fbb3acf1ea05311,
                    0x3f9cabeeab36830c,
                    0x0000000000000000,
                    0x3f9cabeeab36830c,
                    0x3f90283abdb18fd9,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3ff0000000000000,
                ],
                qfrc_bias: &[0xbff0f39ac47a1db2, 0x3fcc67f19830ad51, 0x0000000000000000],
                qacc: &[0x4039a3f1e95df825, 0xc04dc851411a5b2c, 0x0000000000000000],
                qfrc_inverse: &[0x0000000000000000, 0xbc98000000000000, 0x0000000000000000],
            },
        ],
    },
    Case {
        name: "spatial two trees",
        xml: r#"<mujoco><option timestep="0.002"/><worldbody><site name="s0" pos="0.1 0 1.2"/><body name="b1" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s1" pos="0.05 0 -0.1"/><body name="b2" pos="0 0 -0.3"><joint name="j2" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s2" pos="0.05 0 -0.2"/></body></body><body name="c" pos="0.5 0 1"><joint name="k" type="slide" axis="1 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.002 0.002 0.002"/><site name="s3" pos="0 0 0"/></body></worldbody><tendon><spatial name="t" armature="0.4"><site site="s2"/><site site="s3"/></spatial></tendon></mujoco>"#,
        body_simple: &[1, 0, 0, 0],
        dof_simplenum: &[0, 0, 0],
        states: &[
            State {
                qpos: &[0x3fd3333333333333, 0xbfd999999999999a, 0x3fb999999999999a],
                qvel: &[0x3fe999999999999a, 0xbfe0000000000000, 0x3fd3333333333333],
                qm: &[
                    0x3fc5e710a20fd491,
                    0x3fb098437e723eb9,
                    0x0000000000000000,
                    0x3fb098437e723eb9,
                    0x3f9f59ac2ed26128,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3ff3fe1b9ec0c393,
                ],
                qfrc_bias: &[0x3fe28fadc299b460, 0xbfb36b082ffdcbd9, 0x3f8a7a3dce706f62],
                qacc: &[0xc035df79b36cba4a, 0x4048651df04a266a, 0xbf8530990aa6a704],
                qfrc_inverse: &[0x0000000000000000, 0xbca2000000000000, 0x0000000000000000],
            },
            State {
                qpos: &[0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
                qvel: &[0x3ff3333333333333, 0x3fe6666666666666, 0xbfd999999999999a],
                qm: &[
                    0x3fc67eee44ff94d4,
                    0x3fb02b8a3067bc34,
                    0x0000000000000000,
                    0x3fb02b8a3067bc34,
                    0x3f9b9651949f1677,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3ff2dd34e5b15c80,
                ],
                qfrc_bias: &[0xbfba06154aeca84b, 0xbfa7f11dd44a5d64, 0xbfc76bdff6a1caa9],
                qacc: &[0xbfd2780d3cd47fe7, 0x40034c6c3e5a1c5d, 0x3fc3dd892d13e764],
                qfrc_inverse: &[0x0000000000000000, 0xbc60000000000000, 0x0000000000000000],
            },
            State {
                qpos: &[0xbfe3333333333333, 0x3feccccccccccccd, 0xbfc999999999999a],
                qvel: &[0xbfd3333333333333, 0x3ff8000000000000, 0x3feccccccccccccd],
                qm: &[
                    0x3fc170f431e547f2,
                    0x3fa1f0b141c85744,
                    0x0000000000000000,
                    0x3fa1f0b141c85744,
                    0x3f90aaf599b45516,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3ff09238a49c51c2,
                ],
                qfrc_bias: &[0xbfea693f60df0065, 0x3fd0979186c021e8, 0x3fce5ca585ee20fe],
                qacc: &[0x4036be02d90759f0, 0xc0503892dd893da3, 0xbfcd50bd0114c562],
                qfrc_inverse: &[0xbca0000000000000, 0xbca0000000000000, 0x0000000000000000],
            },
        ],
    },
    Case {
        name: "pulley armature",
        xml: r#"<mujoco><option timestep="0.002"/><worldbody><site name="s0" pos="0.1 0 1.2"/><body name="b1" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s1" pos="0.05 0 -0.1"/><body name="b2" pos="0 0 -0.3"><joint name="j2" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s2" pos="0.05 0 -0.2"/></body></body><body name="c" pos="0.5 0 1"><joint name="k" type="slide" axis="1 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.002 0.002 0.002"/><site name="s3" pos="0 0 0"/></body></worldbody><tendon><spatial name="t" armature="0.25"><site site="s0"/><site site="s1"/><pulley divisor="2"/><site site="s0"/><site site="s3"/></spatial></tendon></mujoco>"#,
        body_simple: &[0, 0, 0, 0],
        dof_simplenum: &[0, 0, 0],
        states: &[
            State {
                qpos: &[0x3fd3333333333333, 0xbfd999999999999a, 0x3fb999999999999a],
                qvel: &[0x3fe999999999999a, 0xbfe0000000000000, 0x3fd3333333333333],
                qm: &[
                    0x3fbe128b7fdb9394,
                    0x3fa26b2adaa30982,
                    0x0000000000000000,
                    0x3fa26b2adaa30982,
                    0x3f8f3b645a1cac02,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3ff0dcb08d3dcb09,
                ],
                qfrc_bias: &[0x3fe25b3e55dfcbe8, 0xbfb43d47b5f452a6, 0xbf6f629e0d8cbc3f],
                qacc: &[0xc0374f4e15cc07d4, 0x404e15d9c3c52723, 0x3f6dc7d98199f51c],
                qfrc_inverse: &[0x3cc0000000000000, 0x3ca4000000000000, 0x0000000000000000],
            },
            State {
                qpos: &[0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
                qvel: &[0x3ff3333333333333, 0x3fe6666666666666, 0xbfd999999999999a],
                qm: &[
                    0x3fbf1ff00f0e39af,
                    0x3fa353f7ced91688,
                    0x0000000000000000,
                    0x3fa353f7ced91688,
                    0x3f8f3b645a1cac0a,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3ff0cccccccccccd,
                ],
                qfrc_bias: &[0xbf4e03e5ae779ca7, 0x0000000000000000, 0xbf798404d12cff6a],
                qacc: &[0x3fa0ab695f912e5d, 0xbfb4a1cbdd1a82d3, 0x3f784cf865b0f340],
                qfrc_inverse: &[0x3c00000000000000, 0x0000000000000000, 0x0000000000000000],
            },
            State {
                qpos: &[0xbfe3333333333333, 0x3feccccccccccccd, 0xbfc999999999999a],
                qvel: &[0xbfd3333333333333, 0x3ff8000000000000, 0x3feccccccccccccd],
                qm: &[
                    0x3fbb1b33dea18809,
                    0x3f9df019ccab4aba,
                    0x0000000000000000,
                    0x3f9df019ccab4aba,
                    0x3f8f3b645a1cac06,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3ff0800000000000,
                ],
                qfrc_bias: &[0xbff0901f9711bf9c, 0x3fcc08b0e7087177, 0x3fb01d32b54db337],
                qacc: &[0x403d32ba820dcfd2, 0xc05195a6c5ec5d28, 0xbfaf406257dc825b],
                qfrc_inverse: &[0x0000000000000000, 0x3c98000000000000, 0x0000000000000000],
            },
        ],
    },
    Case {
        name: "no armature control",
        xml: r#"<mujoco><option timestep="0.002"/><worldbody><site name="s0" pos="0.1 0 1.2"/><body name="b1" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s1" pos="0.05 0 -0.1"/><body name="b2" pos="0 0 -0.3"><joint name="j2" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s2" pos="0.05 0 -0.2"/></body></body><body name="c" pos="0.5 0 1"><joint name="k" type="slide" axis="1 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.002 0.002 0.002"/><site name="s3" pos="0 0 0"/></body></worldbody><tendon><spatial name="t"><site site="s0"/><site site="s1"/><site site="s2"/></spatial></tendon></mujoco>"#,
        body_simple: &[1, 0, 0, 2],
        dof_simplenum: &[0, 0, 1],
        states: &[
            State {
                qpos: &[0x3fd3333333333333, 0xbfd999999999999a, 0x3fb999999999999a],
                qvel: &[0x3fe999999999999a, 0xbfe0000000000000, 0x3fd3333333333333],
                qm: &[
                    0x3fbdf04992f4f506,
                    0x3fa26b2adaa30982,
                    0x0000000000000000,
                    0x3fa26b2adaa30982,
                    0x3f8f3b645a1cac02,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3ff0000000000000,
                ],
                qfrc_bias: &[0x3fe25e55ba7978c5, 0xbfb43d47b5f452a6, 0x0000000000000000],
                qacc: &[0xc037b3870c139a15, 0x404e8c0f69c51e62, 0x0000000000000000],
                qfrc_inverse: &[0x3ca0000000000000, 0x3ca4000000000000, 0x0000000000000000],
            },
            State {
                qpos: &[0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
                qvel: &[0x3ff3333333333333, 0x3fe6666666666666, 0xbfd999999999999a],
                qm: &[
                    0x3fbed916872b020e,
                    0x3fa353f7ced91688,
                    0x0000000000000000,
                    0x3fa353f7ced91688,
                    0x3f8f3b645a1cac0a,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3ff0000000000000,
                ],
                qfrc_bias: &[0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
                qacc: &[0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
                qfrc_inverse: &[0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
            },
            State {
                qpos: &[0xbfe3333333333333, 0x3feccccccccccccd, 0xbfc999999999999a],
                qvel: &[0xbfd3333333333333, 0x3ff8000000000000, 0x3feccccccccccccd],
                qm: &[
                    0x3fba7d2b9ea790e2,
                    0x3f9df019ccab4aba,
                    0x0000000000000000,
                    0x3f9df019ccab4aba,
                    0x3f8f3b645a1cac06,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3ff0000000000000,
                ],
                qfrc_bias: &[0xbff0d7be3dab439a, 0x3fcc08b0e7087177, 0x0000000000000000],
                qacc: &[0x403f0d32823e8847, 0xc052790e53a3d37b, 0x0000000000000000],
                qfrc_inverse: &[0xbcc0000000000000, 0xbcb2000000000000, 0x0000000000000000],
            },
        ],
    },
    Case {
        name: "sphere wrap zero",
        xml: r#"<mujoco><option timestep="0.002"/><worldbody><site name="s0" pos="0.1 0 1.2"/><body name="b1" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s1" pos="0.05 0 -0.1"/><body name="b2" pos="0 0 -0.3"><joint name="j2" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s2" pos="0.05 0 -0.2"/></body></body><body name="c" pos="0.5 0 1"><joint name="k" type="slide" axis="1 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.002 0.002 0.002"/><site name="s3" pos="0 0 0"/></body><body name="wb" pos="0.1 0 0.9"><geom name="w" type="sphere" size="0.04"/></body></worldbody><tendon><spatial name="t" armature="0"><site site="s0"/><geom geom="w"/><site site="s2"/></spatial></tendon></mujoco>"#,
        body_simple: &[1, 0, 0, 2, 1],
        dof_simplenum: &[0, 0, 1],
        states: &[
            State {
                qpos: &[0x3fd3333333333333, 0xbfd999999999999a, 0x3fb999999999999a],
                qvel: &[0x3fe999999999999a, 0xbfe0000000000000, 0x3fd3333333333333],
                qm: &[
                    0x3fbdf04992f4f506,
                    0x3fa26b2adaa30982,
                    0x0000000000000000,
                    0x3fa26b2adaa30982,
                    0x3f8f3b645a1cac02,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3ff0000000000000,
                ],
                qfrc_bias: &[0x3fe25e55ba7978c5, 0xbfb43d47b5f452a6, 0x0000000000000000],
                qacc: &[0xc037b3870c139a15, 0x404e8c0f69c51e62, 0x0000000000000000],
                qfrc_inverse: &[0x3ca0000000000000, 0x3ca4000000000000, 0x0000000000000000],
            },
            State {
                qpos: &[0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
                qvel: &[0x3ff3333333333333, 0x3fe6666666666666, 0xbfd999999999999a],
                qm: &[
                    0x3fbed916872b020e,
                    0x3fa353f7ced91688,
                    0x0000000000000000,
                    0x3fa353f7ced91688,
                    0x3f8f3b645a1cac0a,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3ff0000000000000,
                ],
                qfrc_bias: &[0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
                qacc: &[0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
                qfrc_inverse: &[0x0000000000000000, 0x0000000000000000, 0x0000000000000000],
            },
            State {
                qpos: &[0xbfe3333333333333, 0x3feccccccccccccd, 0xbfc999999999999a],
                qvel: &[0xbfd3333333333333, 0x3ff8000000000000, 0x3feccccccccccccd],
                qm: &[
                    0x3fba7d2b9ea790e2,
                    0x3f9df019ccab4aba,
                    0x0000000000000000,
                    0x3f9df019ccab4aba,
                    0x3f8f3b645a1cac06,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3ff0000000000000,
                ],
                qfrc_bias: &[0xbff0d7be3dab439a, 0x3fcc08b0e7087177, 0x0000000000000000],
                qacc: &[0x403f0d32823e8847, 0xc052790e53a3d37b, 0x0000000000000000],
                qfrc_inverse: &[0xbcc0000000000000, 0xbcb2000000000000, 0x0000000000000000],
            },
        ],
    },
    Case {
        name: "spatial free",
        xml: r#"<mujoco><worldbody><site name="s0" pos="0.1 0 1.2"/><body name="b1" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s1" pos="0.05 0 -0.1"/><body name="b2" pos="0 0 -0.3"><joint name="j2" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s2" pos="0.05 0 -0.2"/></body></body><body name="f" pos="0.3 0 0.8"><freejoint/><inertial pos="0 0 0" mass="0.6" diaginertia="0.0006 0.0007 0.0008"/><site name="sf" pos="0.02 0.01 0"/></body></worldbody><tendon><spatial name="t" armature="0.3"><site site="s0"/><site site="sf"/></spatial><spatial name="u" armature="0.15"><site site="s2"/><site site="sf"/></spatial></tendon></mujoco>"#,
        body_simple: &[0, 0, 0, 0],
        dof_simplenum: &[0, 0, 0, 0, 0, 0, 0, 0],
        states: &[
            State {
                qpos: &[
                    0x3fd3333333333333,
                    0xbfd999999999999a,
                    0x3fd3d70a3d70a3d7,
                    0x3f947ae147ae147b,
                    0x3fe947ae147ae148,
                    0x3fed8c554c37f5ac,
                    0x3fba43da0adc6899,
                    0xbfca43da0adc6899,
                    0x3fd3b2e388254e72,
                ],
                qvel: &[
                    0x3fe999999999999a,
                    0xbfe0000000000000,
                    0x3fc999999999999a,
                    0xbfb999999999999a,
                    0x3fd3333333333333,
                    0x3fe6666666666666,
                    0xbfd999999999999a,
                    0x3fe0000000000000,
                ],
                qm: &[
                    0x3fc1682fc75cbc24,
                    0x3fa7b3ab610b75fe,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3fa7b3ab610b75fe,
                    0x3f9557598c3f4b51,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3fe83471068c0f61,
                    0x3f96936d91939fdc,
                    0xbfaaa6b3ccd31c6c,
                    0xbf50dd069c90bf27,
                    0x3f60dd069c90bf1b,
                    0xbf65b3e950d93f9c,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3f96936d91939fdc,
                    0x3fe34ddef6aa5e68,
                    0xbf8c0b41eb0684a7,
                    0xbf2a88282c3b1ea4,
                    0x3f3a88282c3b1e95,
                    0xbf370e8a3a2f1bc8,
                    0x0000000000000000,
                    0x0000000000000000,
                    0xbfaaa6b3ccd31c6c,
                    0xbf8c0b41eb0684a7,
                    0x3fec7db002c99237,
                    0x3f66fb66515ba207,
                    0xbf76fb66515ba200,
                    0xbf08d2c8efd57cc0,
                    0x0000000000000000,
                    0x0000000000000000,
                    0xbf50dd069c90bf27,
                    0xbf2a88282c3b1ea4,
                    0x3f66fb66515ba207,
                    0x3f449c5b321cce3a,
                    0xbf0e662038f37b24,
                    0x3ee2fc8c6bed6bb5,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3f60dd069c90bf1b,
                    0x3f3a88282c3b1e95,
                    0xbf76fb66515ba200,
                    0xbf0e662038f37b24,
                    0x3f4abcca94d72a28,
                    0xbef2fc8c6bed6bd6,
                    0x0000000000000000,
                    0x0000000000000000,
                    0xbf65b3e950d93f9c,
                    0xbf370e8a3a2f1bc8,
                    0xbf08d2c8efd57cc0,
                    0x3ee2fc8c6bed6bb5,
                    0xbef2fc8c6bed6bd6,
                    0x3f4bc9d9d3e94a67,
                ],
                qfrc_bias: &[
                    0x3fe233012f74bf39,
                    0xbfb4f919f0301d45,
                    0x3f959beb77e7d9d4,
                    0x3f72183779361674,
                    0x4017449a015b1594,
                    0xbf478724c5c7cfed,
                    0x3f55b9c52c0444df,
                    0xbf26e03d02ec8e30,
                ],
                qacc: &[
                    0xc036a28f059c6d7d,
                    0x404b1aae93729960,
                    0xbfd63a6ae5d7bbaa,
                    0xbfbcfb5dd7fdeb9f,
                    0xc01bbcd6814929ad,
                    0x403b377136fd241c,
                    0xc0473ffaa4296f62,
                    0xc004c61d80076f11,
                ],
                qfrc_inverse: &[
                    0x3ca0000000000000,
                    0x3ca6000000000000,
                    0xbc97000000000000,
                    0xbc74000000000000,
                    0x3cd0000000000000,
                    0x3c53800000000000,
                    0xbc4a000000000000,
                    0x0000000000000000,
                ],
            },
            State {
                qpos: &[
                    0xbfc999999999999a,
                    0x3fe3333333333333,
                    0x3fd1eb851eb851ec,
                    0xbf9eb851eb851eb8,
                    0x3fea3d70a3d70a3d,
                    0x3fe9dd83c7c9e128,
                    0xbfd62b9586ad0a22,
                    0x3fdd8f7208e6b82e,
                    0x3fbd8f7208e6b82e,
                ],
                qvel: &[
                    0xbfe3333333333333,
                    0x3feccccccccccccd,
                    0xbfd3333333333333,
                    0x3fd999999999999a,
                    0x3fb999999999999a,
                    0xbfe999999999999a,
                    0x3fe3333333333333,
                    0x3fc999999999999a,
                ],
                qm: &[
                    0x3fc0ba8f8ff98124,
                    0x3fa43b7138513a18,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3fa43b7138513a18,
                    0x3f9174cceab7fe47,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3fe6f386bfde1cf8,
                    0xbf8bb1e43e994630,
                    0xbfa488446aab4c20,
                    0x3f42463c620120bb,
                    0xbf52463c620120b8,
                    0xbf607da1d8a92a20,
                    0x0000000000000000,
                    0x0000000000000000,
                    0xbf8bb1e43e994630,
                    0x3fe3405336aae309,
                    0x3f80f903636c5816,
                    0xbf0b2ebb316f2d78,
                    0x3f1b2ebb316f2d72,
                    0x3f2e243f1d79a28d,
                    0x0000000000000000,
                    0x0000000000000000,
                    0xbfa488446aab4c20,
                    0x3f80f903636c5816,
                    0x3fedcc260976fffe,
                    0x3f4d38fdeba67b06,
                    0xbf5d38fdeba67b15,
                    0x3f4029bbaebde95e,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3f42463c620120bb,
                    0xbf0b2ebb316f2d78,
                    0x3f4d38fdeba67b06,
                    0x3f43de63e4a18d8d,
                    0xbeea9cda262d95b1,
                    0xbee57b22e2f20b25,
                    0x0000000000000000,
                    0x0000000000000000,
                    0xbf52463c620120b8,
                    0x3f1b2ebb316f2d72,
                    0xbf5d38fdeba67b15,
                    0xbeea9cda262d95b1,
                    0x3f47c4ed5eea2779,
                    0x3ef57b22e2f20b9a,
                    0x0000000000000000,
                    0x0000000000000000,
                    0xbf607da1d8a92a20,
                    0x3f2e243f1d79a28d,
                    0x3f4029bbaebde95e,
                    0xbee57b22e2f20b25,
                    0x3ef57b22e2f20b9a,
                    0x3f4b59defd1d1e25,
                ],
                qfrc_bias: &[
                    0xbfbde9887ffe4672,
                    0x3fd3452d0125dfad,
                    0x3fbda95bf9f66926,
                    0xbf8ba9d43851101c,
                    0x4017561811c18fa6,
                    0x3f41150a36b7dc83,
                    0xbf502a28abede982,
                    0xbf60a16440a38c59,
                ],
                qacc: &[
                    0x4034d7a7bd37b799,
                    0xc0507e7b7c5e4dd0,
                    0xbfe0eb0bece1162e,
                    0x3fb95b760f2836bf,
                    0xc01950d3779aeaba,
                    0x40217047072cac75,
                    0xc02e0e01cedbf18f,
                    0x401561f9e9c6e51a,
                ],
                qfrc_inverse: &[
                    0xbcb2000000000000,
                    0x3c90000000000000,
                    0x3c94000000000000,
                    0x3c68000000000000,
                    0x0000000000000000,
                    0x3c28000000000000,
                    0x3c30000000000000,
                    0xbc20000000000000,
                ],
            },
        ],
    },
    Case {
        name: "spatial ball",
        xml: r#"<mujoco><worldbody><site name="s0" pos="0.1 0 1.2"/><body name="b1" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s1" pos="0.05 0 -0.1"/><body name="b2" pos="0 0 -0.3"><joint name="j2" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s2" pos="0.05 0 -0.2"/></body></body><body name="bb" pos="-0.3 0 1"><joint name="jb" type="ball"/><inertial pos="0 0 -0.12" mass="0.4" diaginertia="0.003 0.0028 0.0002"/><site name="sb" pos="0.03 0 -0.2"/></body></worldbody><tendon><spatial name="t" armature="0.35"><site site="s0"/><site site="sb"/></spatial></tendon></mujoco>"#,
        body_simple: &[0, 0, 0, 0],
        dof_simplenum: &[0, 0, 0, 0, 0],
        states: &[
            State {
                qpos: &[
                    0x3fd3333333333333,
                    0xbfd999999999999a,
                    0x3fed8c554c37f5ac,
                    0x3fca43da0adc6899,
                    0xbfba43da0adc6899,
                    0x3fd3b2e388254e72,
                ],
                qvel: &[
                    0x3fe999999999999a,
                    0xbfe0000000000000,
                    0x3fe3333333333333,
                    0xbfe6666666666666,
                    0x3fd999999999999a,
                ],
                qm: &[
                    0x3fbdf04992f4f506,
                    0x3fa26b2adaa30982,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3fa26b2adaa30982,
                    0x3f8f3b645a1cac02,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3f854d30365f6351,
                    0x3f6dd9c751741e07,
                    0x3f30220acae40f70,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3f6dd9c751741e07,
                    0x3f910ce121556616,
                    0x3f41e91130df4529,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3f30220acae40f70,
                    0x3f41e91130df4529,
                    0x3f2f0de627fa47c6,
                ],
                qfrc_bias: &[
                    0x3fe25e55ba7978c5,
                    0xbfb43d47b5f452a6,
                    0x3fc358ea4726b9f1,
                    0xbfc2bd27f02ca514,
                    0x3f1a64cc630c22d8,
                ],
                qacc: &[
                    0xc037b3870c139a15,
                    0x404e8c0f69c51e62,
                    0xc032ec78b52cb156,
                    0x402a9ec3aacd3ee3,
                    0xc026f021f95b18e0,
                ],
                qfrc_inverse: &[
                    0x3ca0000000000000,
                    0x3ca4000000000000,
                    0x3c80000000000000,
                    0x0000000000000000,
                    0xbc24000000000000,
                ],
            },
            State {
                qpos: &[
                    0xbfe0000000000000,
                    0x3fc999999999999a,
                    0x3feb9aedba588347,
                    0xbfd4b4324bc26275,
                    0x3fd4b4324bc26275,
                    0xbfcb9aedba588347,
                ],
                qvel: &[
                    0xbfd999999999999a,
                    0x3ff199999999999a,
                    0xbfeccccccccccccd,
                    0x3fd3333333333333,
                    0x3fe999999999999a,
                ],
                qm: &[
                    0x3fbe9e4d4c55fc87,
                    0x3fa3192e94041103,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3fa3192e94041103,
                    0x3f8f3b645a1cac06,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3f8259b720cd2a3b,
                    0x3f467225bd339ab8,
                    0x3eff7c5968c97190,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3f467225bd339ab8,
                    0x3f8654d152a4e30e,
                    0x3f1aef607ca45351,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3eff7c5968c97190,
                    0x3f1aef607ca45351,
                    0x3f2ace049846707f,
                ],
                qfrc_bias: &[
                    0xbff46f11533c658d,
                    0xbfcbbd46f8212a10,
                    0xbfd53b57d23448cf,
                    0x3fc7e8449e9a894e,
                    0xbf20e6ec0ace1179,
                ],
                qacc: &[
                    0x403a567c93f6944b,
                    0xc0491b1604d6e160,
                    0x404340cf9e581ab7,
                    0xc03397f51c817026,
                    0x401348d649a4e0d8,
                ],
                qfrc_inverse: &[
                    0xbcb0000000000000,
                    0xbca0000000000000,
                    0xbca0000000000000,
                    0x3c80000000000000,
                    0x3c0c000000000000,
                ],
            },
        ],
    },
    Case {
        name: "pulley chain",
        xml: r#"<mujoco><worldbody><site name="s0" pos="0.1 0 1.2"/><body name="b1" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s1" pos="0.05 0 -0.1"/><body name="b2" pos="0 0 -0.3"><joint name="j2" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s2" pos="0.05 0 -0.2"/></body></body></worldbody><tendon><spatial name="t" armature="0.25"><site site="s0"/><site site="s1"/><pulley divisor="2"/><site site="s0"/><site site="s2"/></spatial></tendon></mujoco>"#,
        body_simple: &[0, 0, 0],
        dof_simplenum: &[0, 0],
        states: &[
            State {
                qpos: &[0x3fd3333333333333, 0xbfd999999999999a],
                qvel: &[0x3fe999999999999a, 0xbfe0000000000000],
                qm: &[
                    0x3fbe535e927a740d,
                    0x3fa2ed38887954d1,
                    0x3fa2ed38887954d1,
                    0x3f904866e93f7dd0,
                ],
                qfrc_bias: &[0x3fe2509b0572a24d, 0xbfb4855d19030a90],
                qacc: &[0xc03753e5b9508692, 0x404da2cff53c284f],
                qfrc_inverse: &[0x3ca0000000000000, 0x0000000000000000],
            },
            State {
                qpos: &[0xbfe3333333333333, 0x3feccccccccccccd],
                qvel: &[0xbfd3333333333333, 0x3ff8000000000000],
                qm: &[
                    0x3fbc105376bf7096,
                    0x3f9d586b343c4902,
                    0x3f9d586b343c4902,
                    0x3f8f57ed1a4a5eda,
                ],
                qfrc_bias: &[0xbff0ee74121dd1ce, 0x3fcc19c7a81cfdae],
                qacc: &[0x403a4148cbfd94d2, 0xc04fc0ef079c5106],
                qfrc_inverse: &[0x0000000000000000, 0xbc90000000000000],
            },
        ],
    },
    Case {
        name: "same body sites",
        xml: r#"<mujoco><worldbody><site name="s0" pos="0.1 0 1.2"/><body name="b1" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s1" pos="0.05 0 -0.1"/><site name="s1b" pos="-0.04 0.02 -0.25"/><body name="b2" pos="0 0 -0.3"><joint name="j2" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s2" pos="0.05 0 -0.2"/></body></body></worldbody><tendon><spatial name="t" armature="0.3"><site site="s0"/><site site="s1"/><site site="s1b"/><site site="s2"/></spatial></tendon></mujoco>"#,
        body_simple: &[0, 0, 0],
        dof_simplenum: &[0, 0],
        states: &[
            State {
                qpos: &[0x3fd3333333333333, 0xbfd999999999999a],
                qvel: &[0x3fe999999999999a, 0xbfe0000000000000],
                qm: &[
                    0x3fbe19657c09b34a,
                    0x3fa2688f6a1dfce7,
                    0x3fa2688f6a1dfce7,
                    0x3f8f3bb902fe74c3,
                ],
                qfrc_bias: &[0x3fe257f61e49a6a5, 0xbfb43ba9cbb75c10],
                qacc: &[0xc0372625dc94a92a, 0x404de0eb2ef01b36],
                qfrc_inverse: &[0xbcb8000000000000, 0x0000000000000000],
            },
            State {
                qpos: &[0xbfe3333333333333, 0x3feccccccccccccd],
                qvel: &[0xbfd3333333333333, 0x3ff8000000000000],
                qm: &[
                    0x3fbb3acf1ea05311,
                    0x3f9c4341e3f74525,
                    0x3f9c4341e3f74525,
                    0x3f909023b69f7b62,
                ],
                qfrc_bias: &[0xbff0df8887a22e48, 0x3fcc2bec8e8a7411],
                qacc: &[0x4038211306799391, 0xc04b63e29d8b6f1e],
                qfrc_inverse: &[0x0000000000000000, 0x3c80000000000000],
            },
        ],
    },
];

/// One snapshot of a sleep case, every value as its bits.
struct SleepStep {
    /// The mass matrix, row by row.
    qm: &'static [u64],
    qacc: &'static [u64],
    /// MuJoCo's per-tree sleep state: below 0 awake, 0 or above asleep.
    tree_asleep: &'static [i32],
}

/// `(what, document, MuJoCo's snapshots after the reset and a forward pass,
/// then after each of four steps)`. Two trees, a hinge each that gravity
/// loads; the second starts asleep and stays asleep, its acceleration 0 (6.77
/// with sleep disabled, measured). In the first, a tendon with armature on
/// each tree: the sleeping tree's rows keep what they took once. In the
/// second, one tendon over both hinges, awake as the first tree is: MuJoCo
/// adds its armature to the sleeping tree's diagonal again at each step
/// (0.514, 0.759, 1.004, …), as its sleeping rows are not cleared.
const SLEEPS: &[(&str, &str, &[SleepStep])] = &[
    (
        "sleep two trees",
        r#"<mujoco><option><flag sleep="enable"/></option><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0.15 0 0" mass="0.5" diaginertia="0.0002 0.004 0.004"/></body><body name="c" pos="1 0 1" sleep="init"><joint name="jc" type="hinge" axis="0 1 0" ref="0"/><inertial pos="0.15 0 0" mass="0.8" diaginertia="0.0003 0.006 0.006"/></body></worldbody><tendon><fixed name="ta" armature="0.4"><joint joint="ja" coef="1"/></fixed><fixed name="tc" armature="0.6"><joint joint="jc" coef="0.5"/></fixed></tendon></mujoco>"#,
        &[
            SleepStep {
                qm: &[
                    0x3fda9374bc6a7efa,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3fc645a1cac08312,
                ],
                qacc: &[0x3ffc596455b16592, 0x0000000000000000],
                tree_asleep: &[-10, 1],
            },
            SleepStep {
                qm: &[
                    0x3fda9374bc6a7efa,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3fc645a1cac08312,
                ],
                qacc: &[0x3ffc596455b16592, 0x0000000000000000],
                tree_asleep: &[-9, 1],
            },
            SleepStep {
                qm: &[
                    0x3fda9374bc6a7efa,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3fc645a1cac08312,
                ],
                qacc: &[0x3ffc596455ae56bb, 0x0000000000000000],
                tree_asleep: &[-11, 1],
            },
            SleepStep {
                qm: &[
                    0x3fda9374bc6a7efa,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3fc645a1cac08312,
                ],
                qacc: &[0x3ffc59645595e009, 0x0000000000000000],
                tree_asleep: &[-11, 1],
            },
            SleepStep {
                qm: &[
                    0x3fda9374bc6a7efa,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3fc645a1cac08312,
                ],
                qacc: &[0x3ffc596455434f6b, 0x0000000000000000],
                tree_asleep: &[-11, 1],
            },
        ],
    ),
    (
        "sleep cross tree",
        r#"<mujoco><option><flag sleep="enable"/></option><worldbody><body name="a" pos="0 0 1"><joint name="ja" type="hinge" axis="0 1 0"/><inertial pos="0.15 0 0" mass="0.5" diaginertia="0.0002 0.004 0.004"/></body><body name="c" pos="1 0 1" sleep="init"><joint name="jc" type="hinge" axis="0 1 0" ref="0"/><inertial pos="0.15 0 0" mass="0.8" diaginertia="0.0003 0.006 0.006"/></body></worldbody><tendon><fixed name="t" armature="0.5"><joint joint="ja" coef="1"/><joint joint="jc" coef="-0.7"/></fixed></tendon><keyframe><key qpos="0.2 0.7"/></keyframe></mujoco>"#,
        &[
            SleepStep {
                qm: &[
                    0x3fe07ced916872b0,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3fe072b020c49ba5,
                ],
                qacc: &[0x3ff6d8df94ae649d, 0x0000000000000000],
                tree_asleep: &[-10, 1],
            },
            SleepStep {
                qm: &[
                    0x3fe07ced916872b0,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3fe849ba5e353f7c,
                ],
                qacc: &[0x3ff6d8df94ae649d, 0x0000000000000000],
                tree_asleep: &[-9, 1],
            },
            SleepStep {
                qm: &[
                    0x3fe07ced916872b0,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3ff010624dd2f1a9,
                ],
                qacc: &[0x3ff6d8df94accad5, 0x0000000000000000],
                tree_asleep: &[-11, 1],
            },
            SleepStep {
                qm: &[
                    0x3fe07ced916872b0,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3ff3fbe76c8b4394,
                ],
                qacc: &[0x3ff6d8df949ffca0, 0x0000000000000000],
                tree_asleep: &[-11, 1],
            },
            SleepStep {
                qm: &[
                    0x3fe07ced916872b0,
                    0x0000000000000000,
                    0x0000000000000000,
                    0x3ff7e76c8b43957f,
                ],
                qacc: &[0x3ff6d8df9474c4ad, 0x0000000000000000],
                tree_asleep: &[-11, 1],
            },
        ],
    ),
];

fn floats(bits: &[u64]) -> Vec<f64> {
    bits.iter().map(|&b| f64::from_bits(b)).collect()
}

fn bits(values: impl IntoIterator<Item = f64>) -> Vec<u64> {
    values.into_iter().map(f64::to_bits).collect()
}

fn mass_matrix(model: &sim_core::Model, data: &sim_core::Data) -> Vec<u64> {
    bits((0..model.nv * model.nv).map(|k| data.qM[(k / model.nv, k % model.nv)]))
}

#[test]
fn a_tendons_armature_is_mujocos() {
    let mut differ = Vec::new();
    for case in CASES {
        let model = load_model(case.xml).unwrap();
        if model.body_simple != case.body_simple || model.dof_simplenum != case.dof_simplenum {
            differ.push(format!(
                "{}: body_simple {:?}, dof_simplenum {:?}",
                case.name, model.body_simple, model.dof_simplenum
            ));
        }
        let mut data = model.make_data();
        for (s, state) in case.states.iter().enumerate() {
            data.reset(&model);
            data.qpos.copy_from_slice(&floats(state.qpos));
            data.qvel.copy_from_slice(&floats(state.qvel));
            data.forward(&model).unwrap();
            let mut check = |what: &str, got: Vec<u64>, want: &[u64]| {
                if got != want {
                    differ.push(format!(
                        "{} state {s}: {what} {:?} (MuJoCo {:?})",
                        case.name,
                        floats(&got),
                        floats(want)
                    ));
                }
            };
            check("qM", mass_matrix(&model, &data), state.qm);
            check(
                "qfrc_bias",
                bits(data.qfrc_bias.iter().copied()),
                state.qfrc_bias,
            );
            check("qacc", bits(data.qacc.iter().copied()), state.qacc);
            data.inverse(&model).unwrap();
            check(
                "qfrc_inverse",
                bits(data.qfrc_inverse.iter().copied()),
                state.qfrc_inverse,
            );
        }
    }
    assert!(differ.is_empty(), "{}", differ.join("\n"));
}

/// A tendon's armature on a sleeping tree, and across a sleeping and an
/// awake tree, step by step as MuJoCo's: the mass matrix and the
/// accelerations bit for bit, and which trees sleep.
#[test]
fn a_sleeping_trees_tendon_armature_is_mujocos() {
    let mut differ = Vec::new();
    for &(what, xml, steps) in SLEEPS {
        let model = load_model(xml).unwrap();
        let mut data = model.make_data();
        data.forward(&model).unwrap();
        for (k, step) in steps.iter().enumerate() {
            if k > 0 {
                data.step(&model).unwrap();
            }
            let got = (mass_matrix(&model, &data), bits(data.qacc.iter().copied()));
            if got.0 != step.qm || got.1 != step.qacc || data.tree_asleep != step.tree_asleep {
                differ.push(format!(
                    "{what}, after {k} steps: qM {:?} qacc {:?} asleep {:?} (MuJoCo {:?} {:?} {:?})",
                    floats(&got.0),
                    floats(&got.1),
                    data.tree_asleep,
                    floats(step.qm),
                    floats(step.qacc),
                    step.tree_asleep
                ));
            }
        }
    }
    assert!(differ.is_empty(), "{}", differ.join("\n"));
}

/// `(what, document)`: a negative armature on a spatial tendon, which MuJoCo
/// refuses ("tendon armature cannot be negative"), and on a fixed tendon, which
/// MuJoCo loads and sim-mjcf refuses (`D-TENDON-ARMATURE-NEGATIVE`).
const NEGATIVE: &[(&str, &str)] = &[
    (
        "spatial",
        r#"<mujoco><option timestep="0.002"/><worldbody><site name="s0" pos="0.1 0 1.2"/><body name="b1" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s1" pos="0.05 0 -0.1"/><body name="b2" pos="0 0 -0.3"><joint name="j2" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s2" pos="0.05 0 -0.2"/></body></body><body name="c" pos="0.5 0 1"><joint name="k" type="slide" axis="1 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.002 0.002 0.002"/><site name="s3" pos="0 0 0"/></body></worldbody><tendon><spatial name="t" armature="-0.1"><site site="s0"/><site site="s2"/></spatial></tendon></mujoco>"#,
    ),
    (
        "fixed",
        r#"<mujoco><option timestep="0.002"/><worldbody><site name="s0" pos="0.1 0 1.2"/><body name="b1" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s1" pos="0.05 0 -0.1"/><body name="b2" pos="0 0 -0.3"><joint name="j2" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s2" pos="0.05 0 -0.2"/></body></body><body name="c" pos="0.5 0 1"><joint name="k" type="slide" axis="1 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.002 0.002 0.002"/><site name="s3" pos="0 0 0"/></body></worldbody><tendon><fixed name="t" armature="-0.1"><joint joint="j1" coef="1"/></fixed></tendon></mujoco>"#,
    ),
];

/// Armature on a tendon that wraps a sphere (MuJoCo: "geom wrapping not supported by tendon
/// armature").
const WRAP: &str = r#"<mujoco><option timestep="0.002"/><worldbody><site name="s0" pos="0.1 0 1.2"/><body name="b1" pos="0 0 1"><joint name="j1" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s1" pos="0.05 0 -0.1"/><body name="b2" pos="0 0 -0.3"><joint name="j2" type="hinge" axis="0 1 0"/><inertial pos="0 0 -0.15" mass="0.5" diaginertia="0.004 0.004 0.0002"/><site name="s2" pos="0.05 0 -0.2"/></body></body><body name="c" pos="0.5 0 1"><joint name="k" type="slide" axis="1 0 0"/><inertial pos="0 0 0" mass="1" diaginertia="0.002 0.002 0.002"/><site name="s3" pos="0 0 0"/></body><body name="wb" pos="0.1 0 0.9"><geom name="w" type="sphere" size="0.04"/></body></worldbody><tendon><spatial name="t" armature="0.2"><site site="s0"/><geom geom="w"/><site site="s2"/></spatial></tendon></mujoco>"#;

#[test]
fn a_negative_tendon_armature_is_refused() {
    let mut missed = Vec::new();
    for &(what, xml) in NEGATIVE {
        match load_model(xml) {
            Err(e @ MjcfError::InvalidValue { .. })
                if e.to_string()
                    == "invalid armature on <tendon>: tendon armature cannot be negative (tendon 't')" =>
                {}
            Err(other) => missed.push(format!("{what}: {other}")),
            Ok(_) => missed.push(format!("{what}: loads")),
        }
    }
    assert!(missed.is_empty(), "{}", missed.join("\n"));
}

#[test]
fn armature_on_a_tendon_that_wraps_a_geom_is_refused() {
    match load_model(WRAP) {
        Err(e @ MjcfError::InvalidModel { .. })
            if e.to_string() == "geom wrapping not supported by tendon armature (tendon 't')" => {}
        other => panic!("{:?}", other.map(|m| m.ntendon)),
    }
}

/// A model given armature on a tendon that wraps a geom after loading, which
/// MuJoCo's compiler would have refused: the wrapped geom's body is not
/// simple, as `FinalizeSimple` reads the path, and no `Data` is made for it
/// nor its derived fields recomputed, as MuJoCo's forward pass cannot take
/// it (`mj_tendonDot` stops; measured on the oracle: `FatalError` in
/// `mj_forward`).
#[test]
fn a_built_model_with_armature_on_a_wrapping_tendon() {
    let mut model = load_model(&WRAP.replace(r#" armature="0.2""#, "")).unwrap();
    let wrapped = model.body_name_to_id["wb"];
    assert_eq!(model.body_simple[wrapped], 1);
    assert!(model.try_make_data().is_ok());
    model.tendon_armature[0] = 0.2;
    model.compute_simple_bodies();
    assert_eq!(model.body_simple[wrapped], 0);
    let refusal = sim_core::TendonError::ArmatureWrapsGeom { tendon: 0 };
    assert_eq!(
        model.try_make_data().err(),
        Some(sim_core::MakeDataError::Tendon(refusal))
    );
    assert_eq!(
        model.recompute_derived(),
        Err(sim_core::ModelError::Tendon(refusal))
    );
}

/// A model built in code that fills its tendons but not `tendon_armature`
/// (written before the field existed) has no armature: its derived fields
/// recompute, it makes its `Data`, steps, and moves as the same model with
/// zero armature does.
#[test]
fn a_built_model_without_tendon_armature_has_none() {
    let model = load_model(CASES[0].xml.replace(r#" armature="0.7""#, "").as_str()).unwrap();
    assert!(model.tendon_armature.iter().all(|&a| a == 0.0));
    let mut empty = model.clone();
    empty.tendon_armature.clear();
    empty.recompute_derived().unwrap();
    let (mut with, mut without) = (model.make_data(), empty.make_data());
    for _ in 0..5 {
        with.step(&model).unwrap();
        without.step(&empty).unwrap();
    }
    assert_eq!(
        bits(with.qpos.iter().copied()),
        bits(without.qpos.iter().copied())
    );
    assert_eq!(empty.check_tendons(), Ok(()));
}
