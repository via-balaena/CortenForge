//! A height field's data as MuJoCo 3.5.0's compiler makes it (book L45;
//! ledger L42, L48, L208, L210, L356, L359, L428, L431): its rows reversed,
//! so the document's or the image's first row is the field's highest Y; its
//! values normalised in single precision; its grid as written, not
//! resampled; a file typed by its content type or extension, a well-formed
//! PNG decoded to 8-bit grey as lodepng converts it (a malformed one as
//! registry `D-HFIELD-PNG` says), or MuJoCo's own format; and each field
//! compiled in turn after the bodies. A case reading a PNG runs with the
//! `hfields` feature.
//!
//! Every expected value below was printed by MuJoCo 3.5.0 on the census's
//! unfused oracle (2026-11-26 and 2026-11-27) for the same document and
//! files: each height is MuJoCo's `hfield_data` times `size[2]`, as its
//! collision reads it, compared bit for bit, and so are the geom's size and
//! the grid's spacing from MuJoCo's size and counts (single-precision
//! arithmetic and products, the same on every platform). Each refusal gives
//! MuJoCo's reason and this crate's words, which
//! are MuJoCo's reason but for a missing file's and a body's orientation's,
//! `<dir>` the documents' directory
//! (each path's separators read as `/`).

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use std::path::Path;

use sim_mjcf::load_model_from_file;

/// A document loaded by MuJoCo 3.5.0 and its values there.
struct Load {
    what: &'static str,
    xml: &'static str,
    /// Each file the document names: its name and bytes.
    files: &'static [(&'static str, &'static [u8])],
    nrow: usize,
    ncol: usize,
    /// Each height's bits, the field's lowest Y first.
    heights: &'static [u64],
    /// The height-field geom's size's bits.
    geom_size: [u64; 3],
    /// The grid's spacing's bits: `2·size/(n − 1)` along each axis from
    /// MuJoCo's size and counts, the axis's whole width for one row or
    /// column.
    spacing: [u64; 2],
    /// Whether the field is a PNG, which needs the `hfields` feature.
    png: bool,
}

/// A document MuJoCo 3.5.0 refuses: its reason, and this crate's words.
struct Refused {
    what: &'static str,
    xml: &'static str,
    files: &'static [(&'static str, &'static [u8])],
    mujoco: &'static str,
    ours: &'static str,
    /// Whether the field is a PNG, which needs the `hfields` feature.
    png: bool,
}

/// The document written into `dir` with its files, loaded from its path.
fn load(dir: &Path, xml: &str, files: &[(&str, &[u8])]) -> sim_mjcf::Result<sim_core::Model> {
    for (name, bytes) in files {
        std::fs::write(dir.join(name), bytes).unwrap();
    }
    let path = dir.join("model.xml");
    std::fs::write(&path, xml).unwrap();
    load_model_from_file(&path)
}

const LOADS: &[Load] = &[
    Load {
        what: "two rows: the document's first is the field's highest Y",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0.5 0.1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        nrow: 2,
        ncol: 2,
        heights: &[0x3fe0_0000_0000_0000, 0x3fe0_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x4000_0000_0000_0000, 0x4000_0000_0000_0000],
        png: false,
    },
    Load {
        what: "three rows of three",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="3" ncol="3" size="1 1 0.5 0.1" elevation="0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        nrow: 3,
        ncol: 3,
        heights: &[0x3fd8_0000_0000_0000, 0x3fdc_0000_0000_0000, 0x3fe0_0000_0000_0000, 0x3fc8_0000_0000_0000, 0x3fd0_0000_0000_0000, 0x3fd4_0000_0000_0000, 0x0000_0000_0000_0000, 0x3fb0_0000_0000_0000, 0x3fc0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000],
        png: false,
    },
    Load {
        what: "a flat field at 0.5: moved to 0, not divided",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="4" ncol="4" size="1 1 0.5 0.1" elevation="0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        nrow: 4,
        ncol: 4,
        heights: &[0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3fe5_5555_5555_5555, 0x3fe5_5555_5555_5555],
        png: false,
    },
    Load {
        what: "elevations below 0 and above 1, normalised",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="2" ncol="3" size="1 1 0.5 0.1" elevation="-1 0 1 2 -0.5 0.25"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        nrow: 2,
        ncol: 3,
        heights: &[0x3fe0_0000_0000_0000, 0x3fb5_5555_6000_0000, 0x3fca_aaaa_a000_0000, 0x0000_0000_0000_0000, 0x3fc5_5555_6000_0000, 0x3fd5_5555_6000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3ff0_0000_0000_0000, 0x4000_0000_0000_0000],
        png: false,
    },
    Load {
        what: "elevations up to 3, normalised",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="3" ncol="2" size="1 1 0.5 0.1" elevation="0 3 1.5 2 2.5 0.7"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        nrow: 3,
        ncol: 2,
        heights: &[0x3fda_aaaa_a000_0000, 0x3fbd_dddd_e000_0000, 0x3fd0_0000_0000_0000, 0x3fd5_5555_6000_0000, 0x0000_0000_0000_0000, 0x3fe0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x4000_0000_0000_0000, 0x3ff0_0000_0000_0000],
        png: false,
    },
    Load {
        what: "elevations above 1E+10: MuJoCo's least starts there",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0.5 0.1" elevation="2e10 3e10 2e10 3e10"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        nrow: 2,
        ncol: 2,
        heights: &[0x3fcf_ffff_c000_0000, 0x3fe0_0000_0000_0000, 0x3fcf_ffff_c000_0000, 0x3fe0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x4000_0000_0000_0000, 0x4000_0000_0000_0000],
        png: false,
    },
    Load {
        what: "elevations below -1E+10: MuJoCo's greatest starts there",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0.5 0.1" elevation="-3e10 -2e10 -3e10 -2e10"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        nrow: 2,
        ncol: 2,
        heights: &[0x0000_0000_0000_0000, 0x3fd0_0000_0000_0000, 0x0000_0000_0000_0000, 0x3fd0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x4000_0000_0000_0000, 0x4000_0000_0000_0000],
        png: false,
    },
    Load {
        what: "a span of one float step, above mjEPS: divided",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0.5 0.1" elevation="1 1.0000001 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        nrow: 2,
        ncol: 2,
        heights: &[0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x3fe0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x4000_0000_0000_0000, 0x4000_0000_0000_0000],
        png: false,
    },
    Load {
        what: "counts and no elevation: zeros (ledger L359)",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="3" ncol="3" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        nrow: 3,
        ncol: 3,
        heights: &[0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000],
        png: false,
    },
    Load {
        what: "one row (ledger L208)",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="1" ncol="3" size="1 1 0.5 0.1" elevation="0 1 0.5"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        nrow: 1,
        ncol: 3,
        heights: &[0x0000_0000_0000_0000, 0x3fe0_0000_0000_0000, 0x3fd0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3ff0_0000_0000_0000, 0x4000_0000_0000_0000],
        png: false,
    },
    Load {
        what: "one column (ledger L208)",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="3" ncol="1" size="1 1 0.5 0.1" elevation="0 1 0.5"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        nrow: 3,
        ncol: 1,
        heights: &[0x3fd0_0000_0000_0000, 0x3fe0_0000_0000_0000, 0x0000_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x4000_0000_0000_0000, 0x3ff0_0000_0000_0000],
        png: false,
    },
    Load {
        what: "one row of one",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="1" ncol="1" size="1 1 0.5 0.1" elevation="0.5"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        nrow: 1,
        ncol: 1,
        heights: &[0x0000_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x4000_0000_0000_0000, 0x4000_0000_0000_0000],
        png: false,
    },
    Load {
        what: "a half-width of 1e-300 (ledger L210)",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="3" ncol="3" size="1e-300 1 1 1" elevation="0 1 0 1 0 1 0 1 0"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        nrow: 3,
        ncol: 3,
        heights: &[0x0000_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x0000_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x0000_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x0000_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x0000_0000_0000_0000],
        geom_size: [0x01a5_6e1f_c2f8_f359, 0x3ff0_0000_0000_0000, 0x3fe8_0000_0000_0000],
        spacing: [0x01a5_6e1f_c2f8_f359, 0x3ff0_0000_0000_0000],
        png: false,
    },
    Load {
        what: "cells 0.5 by 0.5 on a 9 by 5 grid of half-widths 2 and 1",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="5" ncol="9" size="2 1 0.2 0.1" elevation="0 0.7 0.3 1 0.6 0.2 0.9 0.5 0.1 0.8 0.4 0 0.7 0.3 1 0.6 0.2 0.9 0.5 0.1 0.8 0.4 0 0.7 0.3 1 0.6 0.2 0.9 0.5 0.1 0.8 0.4 0 0.7 0.3 1 0.6 0.2 0.9 0.5 0.1 0.8 0.4 0"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.3 -0.2 0.5"><freejoint/><geom type="sphere" size="0.06" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        nrow: 5,
        ncol: 9,
        heights: &[0x3fc9_9999_9999_999a, 0x3fbe_b852_0000_0000, 0x3fa4_7ae1_4ccc_cccd, 0x3fc7_0a3d_6666_6667, 0x3fb9_9999_9999_999a, 0x3f94_7ae1_4ccc_cccd, 0x3fc4_7ae1_4ccc_cccd, 0x3fb4_7ae1_4ccc_cccd, 0x0000_0000_0000_0000, 0x3fa4_7ae1_4ccc_cccd, 0x3fc7_0a3d_6666_6667, 0x3fb9_9999_9999_999a, 0x3f94_7ae1_4ccc_cccd, 0x3fc4_7ae1_4ccc_cccd, 0x3fb4_7ae1_4ccc_cccd, 0x0000_0000_0000_0000, 0x3fc1_eb85_1999_999a, 0x3fae_b852_0000_0000, 0x3fb9_9999_9999_999a, 0x3f94_7ae1_4ccc_cccd, 0x3fc4_7ae1_4ccc_cccd, 0x3fb4_7ae1_4ccc_cccd, 0x0000_0000_0000_0000, 0x3fc1_eb85_1999_999a, 0x3fae_b852_0000_0000, 0x3fc9_9999_9999_999a, 0x3fbe_b852_0000_0000, 0x3fc4_7ae1_4ccc_cccd, 0x3fb4_7ae1_4ccc_cccd, 0x0000_0000_0000_0000, 0x3fc1_eb85_1999_999a, 0x3fae_b852_0000_0000, 0x3fc9_9999_9999_999a, 0x3fbe_b852_0000_0000, 0x3fa4_7ae1_4ccc_cccd, 0x3fc7_0a3d_6666_6667, 0x0000_0000_0000_0000, 0x3fc1_eb85_1999_999a, 0x3fae_b852_0000_0000, 0x3fc9_9999_9999_999a, 0x3fbe_b852_0000_0000, 0x3fa4_7ae1_4ccc_cccd, 0x3fc7_0a3d_6666_6667, 0x3fb9_9999_9999_999a, 0x3f94_7ae1_4ccc_cccd],
        geom_size: [0x4000_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fb9_9999_9999_999a],
        spacing: [0x3fe0_0000_0000_0000, 0x3fe0_0000_0000_0000],
        png: false,
    },
    Load {
        what: "cells 2 by 3: not resampled (ledger L48)",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="3" ncol="2" size="1 3 0.4 0.1" elevation="0 0.5 1 0.25 0.75 0"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        nrow: 3,
        ncol: 2,
        heights: &[0x3fd3_3333_3333_3334, 0x0000_0000_0000_0000, 0x3fd9_9999_9999_999a, 0x3fb9_9999_9999_999a, 0x0000_0000_0000_0000, 0x3fc9_9999_9999_999a],
        geom_size: [0x3ff0_0000_0000_0000, 0x4008_0000_0000_0000, 0x3fc3_3333_3333_3334],
        spacing: [0x4000_0000_0000_0000, 0x4008_0000_0000_0000],
        png: false,
    },
    Load {
        what: "elevations split by tabs and line breaks",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0.5 0.1" elevation="  0	0
1 1 "/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        nrow: 2,
        ncol: 2,
        heights: &[0x3fe0_0000_0000_0000, 0x3fe0_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x4000_0000_0000_0000, 0x4000_0000_0000_0000],
        png: false,
    },
    Load {
        what: "an empty elevation: zeros",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0.5 0.1" elevation=""/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        nrow: 2,
        ncol: 2,
        heights: &[0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x4000_0000_0000_0000, 0x4000_0000_0000_0000],
        png: false,
    },
    Load {
        what: "a span of 2e-15, not above mjEPS: not divided",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0.5 0.1" elevation="0 1e-15 2e-15 0"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/></worldbody></mujoco>
"#,
        files: &[],
        nrow: 2,
        ncol: 2,
        heights: &[0x3cd2_03af_a000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x3cc2_03af_a000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x4000_0000_0000_0000, 0x4000_0000_0000_0000],
        png: false,
    },
    Load {
        what: "a span of the float nearest 1e-14, below mjEPS: not divided",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0.5 0.1" elevation="0 1e-14 0 0"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        nrow: 2,
        ncol: 2,
        heights: &[0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x3cf6_849b_8000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x4000_0000_0000_0000, 0x4000_0000_0000_0000],
        png: false,
    },
    Load {
        what: "a span of the float nearest 1.0000001e-14, above mjEPS: divided",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0.5 0.1" elevation="0 1.0000001e-14 0 0"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        nrow: 2,
        ncol: 2,
        heights: &[0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x3fe0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x4000_0000_0000_0000, 0x4000_0000_0000_0000],
        png: false,
    },
    Load {
        what: "a span below mjEPS whose least is 1e-15: moved to 0, not divided",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0.5 0.1" elevation="1e-15 2e-15 3e-15 1e-15"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        nrow: 2,
        ncol: 2,
        heights: &[0x3cd2_03af_8000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x3cc2_03af_a000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x4000_0000_0000_0000, 0x4000_0000_0000_0000],
        png: false,
    },
    Load {
        what: "the geom's size: the asset's half-widths and a quarter of its top plus half its base (ledger L356)",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="2" ncol="2" size="1 2 0.5 0.1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/></worldbody></mujoco>"#,
        files: &[],
        nrow: 2,
        ncol: 2,
        heights: &[0x3fe0_0000_0000_0000, 0x3fe0_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x4000_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x4000_0000_0000_0000, 0x4010_0000_0000_0000],
        png: false,
    },
    Load {
        what: "an 8-bit grey PNG, its rows reversed",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="g8.png" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("g8.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x04\x00\x00\x00\x03\x08\x00\x00\x00\x00\x91\x9f\xf1\x1a\x00\x00\x00\x17IDATx\x9cc`ph\xf8\xcf\xc0%\x22\xa7\xc1p\x22\xc5H\x12\x00\x1b\xc9\x03\x9b\xdc\x00\x07\xa4\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 3,
        ncol: 4,
        heights: &[0x3fd9_1919_2000_0000, 0x3fc9_1919_2000_0000, 0x3fb9_1919_2000_0000, 0x3fa9_1919_2000_0000, 0x3f94_1414_2000_0000, 0x3fa4_1414_2000_0000, 0x3fae_1e1e_2000_0000, 0x3fb4_1414_2000_0000, 0x0000_0000_0000_0000, 0x3fc0_1010_2000_0000, 0x3fd0_1010_2000_0000, 0x3fe0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3fe5_5555_5555_5555, 0x3ff0_0000_0000_0000],
        png: true,
    },
    Load {
        what: "a 16-bit grey PNG: each sample's high byte",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="g16.png" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("g16.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x04\x00\x00\x00\x03\x10\x00\x00\x00\x00\xc1\x0f-Y\x00\x00\x00$IDATx\x9cc``p\xf8\xdf\xc0\xf8\xff?\x03\x17\x97\xc8\x7f9\x06\x8d\x06\x86\x13'R\x18\x8d\xfeIJ\x02\x00y\xba\x09\x03\x9b\xe4\xe89\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 3,
        ncol: 4,
        heights: &[0x3fd9_1919_2000_0000, 0x3fc9_1919_2000_0000, 0x3fb9_1919_2000_0000, 0x3fa9_1919_2000_0000, 0x3f94_1414_2000_0000, 0x3fa4_1414_2000_0000, 0x3fae_1e1e_2000_0000, 0x3fb4_1414_2000_0000, 0x0000_0000_0000_0000, 0x3fc0_1010_2000_0000, 0x3fd0_1010_2000_0000, 0x3fe0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3fe5_5555_5555_5555, 0x3ff0_0000_0000_0000],
        png: true,
    },
    Load {
        what: "an RGB PNG: each pixel's red",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="rgb.png" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("rgb.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x04\x00\x00\x00\x03\x08\x02\x00\x00\x00;\x969\x91\x00\x00\x00/IDATx\x9cc`\xf8\xcf\xee\xc0\xc8\xd4\xc0\xc9\xf9\x9f\x81\x81\x81\xeb\x04\xb3\x08\x0b\xab\x1c\x1b\xbb\x06\x07'\xc3\x09\x06\x86\x14#I###\xc9_\xb5\x00\x8f\xf9\x07\xce\x88y\xf2\xcc\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 3,
        ncol: 4,
        heights: &[0x3fd9_1919_2000_0000, 0x3fc9_1919_2000_0000, 0x3fb9_1919_2000_0000, 0x3fa9_1919_2000_0000, 0x3f94_1414_2000_0000, 0x3fa4_1414_2000_0000, 0x3fae_1e1e_2000_0000, 0x3fb4_1414_2000_0000, 0x0000_0000_0000_0000, 0x3fc0_1010_2000_0000, 0x3fd0_1010_2000_0000, 0x3fe0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3fe5_5555_5555_5555, 0x3ff0_0000_0000_0000],
        png: true,
    },
    Load {
        what: "an RGBA PNG: each pixel's red",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="rgba.png" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("rgba.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x04\x00\x00\x00\x03\x08\x06\x00\x00\x00\xb4\xf4\xae\xc6\x00\x00\x00;IDATx\x9cc`\xf8\xcf\xce\xe0\xc0\xc8\xf4\xbf\x81\x93\xb3\xe1?\x03\x033\x03\xd7\x09fN\x11\x16VN96vN\x0d\x0eNN\x86\x13\x0c\x0c\x9c)F\x92\x9cFFF\x9c\x92\xbfj9\x01\x02}\x09\x98\xaaV\x0b\xee\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 3,
        ncol: 4,
        heights: &[0x3fd9_1919_2000_0000, 0x3fc9_1919_2000_0000, 0x3fb9_1919_2000_0000, 0x3fa9_1919_2000_0000, 0x3f94_1414_2000_0000, 0x3fa4_1414_2000_0000, 0x3fae_1e1e_2000_0000, 0x3fb4_1414_2000_0000, 0x0000_0000_0000_0000, 0x3fc0_1010_2000_0000, 0x3fd0_1010_2000_0000, 0x3fe0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3fe5_5555_5555_5555, 0x3ff0_0000_0000_0000],
        png: true,
    },
    Load {
        what: "a grey and alpha PNG: its grey",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="ga.png" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("ga.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x04\x00\x00\x00\x03\x08\x04\x00\x00\x00\x1e\xfdfM\x00\x00\x00$IDATx\x9cc`\xf8\xef\xb0\xbf\xa1\xfe?\x03\x03\xd7W\x91\xd7r\x0f5\xae3\x9c0O\x99mtV\xf2\x19\x00\x9f\x19\x0b\xf5\x13\x1c\xea\xc1\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 3,
        ncol: 4,
        heights: &[0x3fd9_1919_2000_0000, 0x3fc9_1919_2000_0000, 0x3fb9_1919_2000_0000, 0x3fa9_1919_2000_0000, 0x3f94_1414_2000_0000, 0x3fa4_1414_2000_0000, 0x3fae_1e1e_2000_0000, 0x3fb4_1414_2000_0000, 0x0000_0000_0000_0000, 0x3fc0_1010_2000_0000, 0x3fd0_1010_2000_0000, 0x3fe0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3fe5_5555_5555_5555, 0x3ff0_0000_0000_0000],
        png: true,
    },
    Load {
        what: "a palette PNG: each entry's red",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="pal.png" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("pal.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x04\x00\x00\x00\x03\x08\x03\x00\x00\x00\x83*^\xf4\x00\x00\x00\x0cPLTE\x00\x09\x09Z\x00\x00\xb4\xff\xff\xff\x01\x01\x10sL\x15\x00\x00\x00\x17IDATx\x9cc``dbf`fbd``ddb\x02\x00\x00\x94\x00\x13$\xc5\xffa\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 3,
        ncol: 4,
        heights: &[0x3fc6_9696_a000_0000, 0x3fc6_9696_a000_0000, 0x3fd6_9696_a000_0000, 0x3fd6_9696_a000_0000, 0x3fe0_0000_0000_0000, 0x3fd6_9696_a000_0000, 0x3fc6_9696_a000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x3fc6_9696_a000_0000, 0x3fd6_9696_a000_0000, 0x3fe0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3fe5_5555_5555_5555, 0x3ff0_0000_0000_0000],
        png: true,
    },
    Load {
        what: "a PNG named `.png` (ledger L428)",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file=".png" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[(".png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x03\x00\x00\x00\x03\x08\x00\x00\x00\x00sC\xeac\x00\x00\x00\x14IDATx\x9cc`0Ja\x98v\xe2\x17\x03+\xbf$\x00\x13v\x03\x1cns\xf1\xa2\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 3,
        ncol: 3,
        heights: &[0x3f84_7ae1_4000_0000, 0x3f9e_b851_e000_0000, 0x3fa9_9999_a000_0000, 0x3fd3_3333_4000_0000, 0x3fd9_9999_a000_0000, 0x3fe0_0000_0000_0000, 0x0000_0000_0000_0000, 0x3fb9_9999_a000_0000, 0x3fc9_9999_a000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000],
        png: true,
    },
    Load {
        what: "a PNG named `H03.PNG`",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="H03.PNG" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("H03.PNG", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x03\x00\x00\x00\x03\x08\x00\x00\x00\x00sC\xeac\x00\x00\x00\x14IDATx\x9cc`0Ja\x98v\xe2\x17\x03+\xbf$\x00\x13v\x03\x1cns\xf1\xa2\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 3,
        ncol: 3,
        heights: &[0x3f84_7ae1_4000_0000, 0x3f9e_b851_e000_0000, 0x3fa9_9999_a000_0000, 0x3fd3_3333_4000_0000, 0x3fd9_9999_a000_0000, 0x3fe0_0000_0000_0000, 0x0000_0000_0000_0000, 0x3fb9_9999_a000_0000, 0x3fc9_9999_a000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000],
        png: true,
    },
    Load {
        what: "content_type image/png on another extension (ledger L428)",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="h_img.img" content_type="image/png" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("h_img.img", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x03\x00\x00\x00\x03\x08\x00\x00\x00\x00sC\xeac\x00\x00\x00\x14IDATx\x9cc`0Ja\x98v\xe2\x17\x03+\xbf$\x00\x13v\x03\x1cns\xf1\xa2\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 3,
        ncol: 3,
        heights: &[0x3f84_7ae1_4000_0000, 0x3f9e_b851_e000_0000, 0x3fa9_9999_a000_0000, 0x3fd3_3333_4000_0000, 0x3fd9_9999_a000_0000, 0x3fe0_0000_0000_0000, 0x0000_0000_0000_0000, 0x3fb9_9999_a000_0000, 0x3fc9_9999_a000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000],
        png: true,
    },
    Load {
        what: "MuJoCo's own format, named `.bin`: an extension of no type",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="h.bin" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("h.bin", b"\x02\x00\x00\x00\x03\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80?\x00\x00\x00@\x00\x00\x00?\x00\x00\x80\xbf\x00\x00\x80@")],
        nrow: 2,
        ncol: 3,
        heights: &[0x3fb9_9999_a000_0000, 0x3fc9_9999_a000_0000, 0x3fd3_3333_4000_0000, 0x3fc3_3333_4000_0000, 0x0000_0000_0000_0000, 0x3fe0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3ff0_0000_0000_0000, 0x4000_0000_0000_0000],
        png: false,
    },
    Load {
        what: "MuJoCo's own format by its content type",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="h.dat" content_type="image/vnd.mujoco.hfield" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("h.dat", b"\x03\x00\x00\x00\x02\x00\x00\x00\x00\x00\x80>\x00\x00\x00?\x00\x00@?\x00\x00\x80?\x00\x00\x00\x00\x00\x00\x00>")],
        nrow: 3,
        ncol: 2,
        heights: &[0x3fc0_0000_0000_0000, 0x3fd0_0000_0000_0000, 0x3fd8_0000_0000_0000, 0x3fe0_0000_0000_0000, 0x0000_0000_0000_0000, 0x3fb0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x4000_0000_0000_0000, 0x3ff0_0000_0000_0000],
        png: false,
    },
    Load {
        what: "a file's written counts not read",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="g8.png" nrow="7" ncol="7" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("g8.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x04\x00\x00\x00\x03\x08\x00\x00\x00\x00\x91\x9f\xf1\x1a\x00\x00\x00\x17IDATx\x9cc`ph\xf8\xcf\xc0%\x22\xa7\xc1p\x22\xc5H\x12\x00\x1b\xc9\x03\x9b\xdc\x00\x07\xa4\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 3,
        ncol: 4,
        heights: &[0x3fd9_1919_2000_0000, 0x3fc9_1919_2000_0000, 0x3fb9_1919_2000_0000, 0x3fa9_1919_2000_0000, 0x3f94_1414_2000_0000, 0x3fa4_1414_2000_0000, 0x3fae_1e1e_2000_0000, 0x3fb4_1414_2000_0000, 0x0000_0000_0000_0000, 0x3fc0_1010_2000_0000, 0x3fd0_1010_2000_0000, 0x3fe0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3fe5_5555_5555_5555, 0x3ff0_0000_0000_0000],
        png: true,
    },
    Load {
        what: "a PNG of one pixel",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="one.png" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("one.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x01\x00\x00\x00\x01\x08\x00\x00\x00\x00:~\x9bU\x00\x00\x00\x0aIDATx\x9cc\xf0\x05\x00\x00O\x00Ni\x8b\x01l\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 1,
        ncol: 1,
        heights: &[0x0000_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x4000_0000_0000_0000, 0x4000_0000_0000_0000],
        png: true,
    },
    Load {
        what: "a file and an elevation: the file's data (ledger L431)",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="g8.png" nrow="2" ncol="2" size="1 1 0.5 0.1" elevation="0 0 0 0"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("g8.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x04\x00\x00\x00\x03\x08\x00\x00\x00\x00\x91\x9f\xf1\x1a\x00\x00\x00\x17IDATx\x9cc`ph\xf8\xcf\xc0%\x22\xa7\xc1p\x22\xc5H\x12\x00\x1b\xc9\x03\x9b\xdc\x00\x07\xa4\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 3,
        ncol: 4,
        heights: &[0x3fd9_1919_2000_0000, 0x3fc9_1919_2000_0000, 0x3fb9_1919_2000_0000, 0x3fa9_1919_2000_0000, 0x3f94_1414_2000_0000, 0x3fa4_1414_2000_0000, 0x3fae_1e1e_2000_0000, 0x3fb4_1414_2000_0000, 0x0000_0000_0000_0000, 0x3fc0_1010_2000_0000, 0x3fd0_1010_2000_0000, 0x3fe0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3fe5_5555_5555_5555, 0x3ff0_0000_0000_0000],
        png: true,
    },
    Load {
        what: "MuJoCo's own format, one value",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="one.bin" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("one.bin", b"\x01\x00\x00\x00\x01\x00\x00\x00\x00\x00@@")],
        nrow: 1,
        ncol: 1,
        heights: &[0x0000_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x4000_0000_0000_0000, 0x4000_0000_0000_0000],
        png: false,
    },
    Load {
        what: "a flat PNG: zeros",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="flat.png" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("flat.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x03\x00\x00\x00\x02\x08\x00\x00\x00\x00\xb8\x1f9\xc6\x00\x00\x00\x0eIDATx\x9cc\xe0\xe4\xe4d\x00b\x00\x00\xe0\x007\xf9!\x81\x88\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 2,
        ncol: 3,
        heights: &[0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3ff0_0000_0000_0000, 0x4000_0000_0000_0000],
        png: true,
    },
    Load {
        what: "a content type with a parameter",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="h_img.img" content_type="image/png; q=1" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("h_img.img", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x03\x00\x00\x00\x03\x08\x00\x00\x00\x00sC\xeac\x00\x00\x00\x14IDATx\x9cc`0Ja\x98v\xe2\x17\x03+\xbf$\x00\x13v\x03\x1cns\xf1\xa2\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 3,
        ncol: 3,
        heights: &[0x3f84_7ae1_4000_0000, 0x3f9e_b851_e000_0000, 0x3fa9_9999_a000_0000, 0x3fd3_3333_4000_0000, 0x3fd9_9999_a000_0000, 0x3fe0_0000_0000_0000, 0x0000_0000_0000_0000, 0x3fb9_9999_a000_0000, 0x3fc9_9999_a000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000],
        png: true,
    },
    Load {
        what: "a 4-bit grey PNG",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="g4.png" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("g4.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x05\x00\x00\x00\x02\x04\x00\x00\x00\x00p\xf1\xa4\x80\x00\x00\x00\x10IDATx\x9cc`\xae\xf9\xc0 4/\x00\x00\x09w\x02p\xf8\xeb4v\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 2,
        ncol: 5,
        heights: &[0x3fa1_1111_2000_0000, 0x3fb1_1111_2000_0000, 0x3fd3_3333_4000_0000, 0x3fdd_dddd_e000_0000, 0x3fc5_5555_6000_0000, 0x0000_0000_0000_0000, 0x3fb9_9999_a000_0000, 0x3fcd_dddd_e000_0000, 0x3fd9_9999_a000_0000, 0x3fe0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3fe0_0000_0000_0000, 0x4000_0000_0000_0000],
        png: true,
    },
    Load {
        what: "a 2-bit grey PNG",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="g2.png" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("g2.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x03\x00\x00\x00\x03\x02\x00\x00\x00\x009\xf3\xf2\xc2\x00\x00\x00\x0eIDATx\x9cc\x90`x\xc2P\x00\x00\x03\x9a\x01m\x17\xf1\xa9\x04\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 3,
        ncol: 3,
        heights: &[0x3fc5_5555_6000_0000, 0x3fe0_0000_0000_0000, 0x0000_0000_0000_0000, 0x3fe0_0000_0000_0000, 0x3fd5_5555_6000_0000, 0x3fc5_5555_6000_0000, 0x0000_0000_0000_0000, 0x3fc5_5555_6000_0000, 0x3fd5_5555_6000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000],
        png: true,
    },
    Load {
        what: "a 16-bit RGB PNG: its red's high byte",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="rgb16.png" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("rgb16.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x03\x00\x00\x00\x02\x10\x02\x00\x00\x00B\x86-\x0e\x00\x00\x00'IDATx\x9cc`d\xfa\xff\x9f\x81\xa1\xe1?\x03#\x03\x13\x90\x01\x02B&a\x15\xb3\xf60\x00\xc5\xf9\xf9\xeb\x1b@\x22\x00\xcaE\x0a\x08\xf3\xa4%z\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 2,
        ncol: 3,
        heights: &[0x3fa2_1212_2000_0000, 0x0000_0000_0000_0000, 0x3fcf_dfdf_e000_0000, 0x3f60_1010_2000_0000, 0x3fd0_1010_2000_0000, 0x3fe0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3ff0_0000_0000_0000, 0x4000_0000_0000_0000],
        png: true,
    },
    Load {
        what: "a 16-bit RGBA PNG",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="rgba16.png" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("rgba16.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x03\x00\x00\x00\x02\x10\x06\x00\x00\x00\xcd\xe4\xbaY\x00\x00\x00/IDATx\x9cc`d\xfa\xff\x9f\x01\x08\x1a\xfe30201p\x829\x0c@1!\x93\xb0\x8aY{\x80\x82@6??\x03S}\x03X\x86\x19\x00H\x85\x0c\x15e\xb7\xa2+\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 2,
        ncol: 3,
        heights: &[0x3fa2_1212_2000_0000, 0x0000_0000_0000_0000, 0x3fcf_dfdf_e000_0000, 0x3f60_1010_2000_0000, 0x3fd0_1010_2000_0000, 0x3fe0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3ff0_0000_0000_0000, 0x4000_0000_0000_0000],
        png: true,
    },
    Load {
        what: "MuJoCo's own format holding a NaN: loaded, the NaN kept",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="nan.bin" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("nan.bin", b"\x02\x00\x00\x00\x02\x00\x00\x00\x00\x00\x00\x00\x00\x00\xc0\x7f\x00\x00\x80?\x00\x00\x00@")],
        nrow: 2,
        ncol: 2,
        heights: &[0x0000_0000_0000_0000, 0x7ff8_0000_0000_0000, 0x3fd0_0000_0000_0000, 0x3fe0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x4000_0000_0000_0000, 0x4000_0000_0000_0000],
        png: false,
    },
    Load {
        what: "a file with no extension: MuJoCo's own format",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="noext" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("noext", b"\x02\x00\x00\x00\x02\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80?\x00\x00\x00@\x00\x00@@")],
        nrow: 2,
        ncol: 2,
        heights: &[0x0000_0000_0000_0000, 0x3fc5_5555_6000_0000, 0x3fd5_5555_6000_0000, 0x3fe0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x4000_0000_0000_0000, 0x4000_0000_0000_0000],
        png: false,
    },
    Load {
        what: "a file beside an elevation that is not a number: the elevation not read",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="g8.png" nrow="2" ncol="2" size="1 1 0.5 0.1" elevation="x"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("g8.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x04\x00\x00\x00\x03\x08\x00\x00\x00\x00\x91\x9f\xf1\x1a\x00\x00\x00\x17IDATx\x9cc`ph\xf8\xcf\xc0%\x22\xa7\xc1p\x22\xc5H\x12\x00\x1b\xc9\x03\x9b\xdc\x00\x07\xa4\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 3,
        ncol: 4,
        heights: &[0x3fd9_1919_2000_0000, 0x3fc9_1919_2000_0000, 0x3fb9_1919_2000_0000, 0x3fa9_1919_2000_0000, 0x3f94_1414_2000_0000, 0x3fa4_1414_2000_0000, 0x3fae_1e1e_2000_0000, 0x3fb4_1414_2000_0000, 0x0000_0000_0000_0000, 0x3fc0_1010_2000_0000, 0x3fd0_1010_2000_0000, 0x3fe0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3fe5_5555_5555_5555, 0x3ff0_0000_0000_0000],
        png: true,
    },
    Load {
        what: "an empty content type: the extension's",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="g8.png" content_type="" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("g8.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x04\x00\x00\x00\x03\x08\x00\x00\x00\x00\x91\x9f\xf1\x1a\x00\x00\x00\x17IDATx\x9cc`ph\xf8\xcf\xc0%\x22\xa7\xc1p\x22\xc5H\x12\x00\x1b\xc9\x03\x9b\xdc\x00\x07\xa4\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 3,
        ncol: 4,
        heights: &[0x3fd9_1919_2000_0000, 0x3fc9_1919_2000_0000, 0x3fb9_1919_2000_0000, 0x3fa9_1919_2000_0000, 0x3f94_1414_2000_0000, 0x3fa4_1414_2000_0000, 0x3fae_1e1e_2000_0000, 0x3fb4_1414_2000_0000, 0x0000_0000_0000_0000, 0x3fc0_1010_2000_0000, 0x3fd0_1010_2000_0000, 0x3fe0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3fe5_5555_5555_5555, 0x3ff0_0000_0000_0000],
        png: true,
    },
    Load {
        what: "a 1-bit grey PNG",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="grey1.png" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("grey1.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x04\x00\x00\x00\x03\x01\x00\x00\x00\x00\x9c\x8f\x93k\x00\x00\x00\x0eIDATx\x9ccH`\x98\xc0p\x00\x00\x04V\x01\xb1\x9f\xc4\xd6\x92\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 3,
        ncol: 4,
        heights: &[0x3fe0_0000_0000_0000, 0x3fe0_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x3fe0_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x3fe0_0000_0000_0000, 0x0000_0000_0000_0000, 0x3fe0_0000_0000_0000, 0x3fe0_0000_0000_0000, 0x0000_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3fe5_5555_5555_5555, 0x3ff0_0000_0000_0000],
        png: true,
    },
    Load {
        what: "an 8-bit grey PNG with a tRNS chunk",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="grey8_trns.png" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("grey8_trns.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x04\x00\x00\x00\x03\x08\x00\x00\x00\x00\x91\x9f\xf1\x1a\x00\x00\x00\x02tRNS\x00@\x00O\x8c\xa8\x00\x00\x00\x17IDATx\x9cc`ph\xf8\xcf\xc0%\xe2\xa0\xc1\xf0\x9f\xc1\x81\x1d\x00\x1cq\x03\x8c\x13\x81\x00\xfd\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 3,
        ncol: 4,
        heights: &[0x3fe0_0000_0000_0000, 0x0000_0000_0000_0000, 0x3fc0_1010_2000_0000, 0x3f8c_1c1c_2000_0000, 0x3f94_1414_2000_0000, 0x3fa4_1414_2000_0000, 0x3fc0_1010_2000_0000, 0x3fb4_1414_2000_0000, 0x0000_0000_0000_0000, 0x3fc0_1010_2000_0000, 0x3fd0_1010_2000_0000, 0x3fe0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3fe5_5555_5555_5555, 0x3ff0_0000_0000_0000],
        png: true,
    },
    Load {
        what: "a 16-bit grey and alpha PNG: each grey sample's high byte",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="grey_alpha16.png" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("grey_alpha16.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x03\x00\x00\x00\x02\x10\x04\x00\x00\x00g\xedr\xd2\x00\x00\x00#IDATx\x9cc\x102\xf9\xff\xff?#\x03C\xc3\x7fAA\x06\x86\xffJJ\xf5\x0c\xff\xff\xaf>\xcb\xc0\x00\x00\x84f\x0a\x1e\xf1=\xe8\xfc\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 2,
        ncol: 3,
        heights: &[0x0000_0000_0000_0000, 0x3fcf_dfdf_e000_0000, 0x3fd5_7575_8000_0000, 0x3fa2_1212_2000_0000, 0x3fe0_0000_0000_0000, 0x3fd0_1010_2000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3ff0_0000_0000_0000, 0x4000_0000_0000_0000],
        png: true,
    },
    Load {
        what: "a 2-bit palette PNG",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="palette2.png" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("palette2.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x04\x00\x00\x00\x03\x02\x03\x00\x00\x00\xc9\x9aFU\x00\x00\x00\x0cPLTE\x0a\xc8\x1e\xfa\x05\x05<<\xf0\x80\x80\x80B\xd2OC\x00\x00\x00\x0eIDATx\x9cc\x90fx\xc2\x10\x03\x00\x03\x95\x01\x5c\x82\xf2\x99\x93\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 3,
        ncol: 4,
        heights: &[0x3fe0_0000_0000_0000, 0x3fe0_0000_0000_0000, 0x3fcf_7777_8000_0000, 0x0000_0000_0000_0000, 0x3fcf_7777_8000_0000, 0x3fba_aaaa_a000_0000, 0x3fe0_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x3fe0_0000_0000_0000, 0x3fba_aaaa_a000_0000, 0x3fcf_7777_8000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3fe5_5555_5555_5555, 0x3ff0_0000_0000_0000],
        png: true,
    },
    Load {
        what: "a 4-bit palette PNG with a tRNS chunk",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="palette4_trns.png" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("palette4_trns.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x04\x00\x00\x00\x03\x04\x03\x00\x00\x00F\xda\xb3\xf5\x00\x00\x00\x0fPLTE\x0a\xc8\x1e\xfa\x05\x05<<\xf0\x80\x80\x80M\x01\x02\x1d\xf6D\x05\x00\x00\x00\x03tRNS\x00\x80\xff\xec\xf7\xb3\x18\x00\x00\x00\x11IDATx\x9cc`Tfp\x12`\x101\x00\x00\x02\xe8\x00\xbb\x9d\xd4U\xf8\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 3,
        ncol: 4,
        heights: &[0x3fe0_0000_0000_0000, 0x3fc1_dddd_e000_0000, 0x3fcf_7777_8000_0000, 0x0000_0000_0000_0000, 0x3fc1_dddd_e000_0000, 0x3fba_aaaa_a000_0000, 0x3fe0_0000_0000_0000, 0x0000_0000_0000_0000, 0x0000_0000_0000_0000, 0x3fe0_0000_0000_0000, 0x3fba_aaaa_a000_0000, 0x3fcf_7777_8000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3fe5_5555_5555_5555, 0x3ff0_0000_0000_0000],
        png: true,
    },
    Load {
        what: "an RGB PNG with gAMA and sRGB chunks",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="rgb_ancillary.png" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("rgb_ancillary.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x03\x00\x00\x00\x02\x08\x02\x00\x00\x00\x12\x16\xf1M\x00\x00\x00\x04gAMA\x00\x00\xb1\x8f\x0b\xfca\x05\x00\x00\x00\x01sRGB\x00\xae\xce\x1c\xe9\x00\x00\x00\x19IDATx\x9cc8\xc1\xc8\xc4\x9c\xc2bdd\xc4\xc0\xf0\x1f\x04\x05\x95\x8c\x01/\x04\x050.8o\x81\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 2,
        ncol: 3,
        heights: &[0x0000_0000_0000_0000, 0x3fe0_0000_0000_0000, 0x3fa1_1111_2000_0000, 0x3fd9_1919_2000_0000, 0x3f78_1818_2000_0000, 0x3fb9_1919_2000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3ff0_0000_0000_0000, 0x4000_0000_0000_0000],
        png: true,
    },
    Load {
        what: "a PNG named `h.x.png`: the extension from the last `.`",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="h.x.png" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("h.x.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x04\x00\x00\x00\x03\x08\x00\x00\x00\x00\x91\x9f\xf1\x1a\x00\x00\x00\x17IDATx\x9cc`ph\xf8\xcf\xc0%\x22\xa7\xc1p\x22\xc5H\x12\x00\x1b\xc9\x03\x9b\xdc\x00\x07\xa4\x00\x00\x00\x00IEND\xaeB`\x82")],
        nrow: 3,
        ncol: 4,
        heights: &[0x3fd9_1919_2000_0000, 0x3fc9_1919_2000_0000, 0x3fb9_1919_2000_0000, 0x3fa9_1919_2000_0000, 0x3f94_1414_2000_0000, 0x3fa4_1414_2000_0000, 0x3fae_1e1e_2000_0000, 0x3fb4_1414_2000_0000, 0x0000_0000_0000_0000, 0x3fc0_1010_2000_0000, 0x3fd0_1010_2000_0000, 0x3fe0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3fe5_5555_5555_5555, 0x3ff0_0000_0000_0000],
        png: true,
    },
    Load {
        what: "MuJoCo's own format named `.png`, by its content type",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="custom.png" content_type="image/vnd.mujoco.hfield" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("custom.png", b"\x02\x00\x00\x00\x03\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80?\x00\x00\x00@\x00\x00\x00?\x00\x00\x80\xbf\x00\x00\x80@")],
        nrow: 2,
        ncol: 3,
        heights: &[0x3fb9_9999_a000_0000, 0x3fc9_9999_a000_0000, 0x3fd3_3333_4000_0000, 0x3fc3_3333_4000_0000, 0x0000_0000_0000_0000, 0x3fe0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3ff0_0000_0000_0000, 0x4000_0000_0000_0000],
        png: false,
    },
    Load {
        what: "a file beside counts whose product passes a C int: the counts not read",
        xml: r#"<mujoco><asset><hfield name="h" file="h.bin" nrow="65536" ncol="65536" size="1 1 0.5 0.1"/></asset><worldbody><geom type="hfield" hfield="h"/></worldbody></mujoco>"#,
        files: &[("h.bin", b"\x02\x00\x00\x00\x03\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80?\x00\x00\x00@\x00\x00\x00?\x00\x00\x80\xbf\x00\x00\x80@")],
        nrow: 2,
        ncol: 3,
        heights: &[0x3fb9_9999_a000_0000, 0x3fc9_9999_a000_0000, 0x3fd3_3333_4000_0000, 0x3fc3_3333_4000_0000, 0x0000_0000_0000_0000, 0x3fe0_0000_0000_0000],
        geom_size: [0x3ff0_0000_0000_0000, 0x3ff0_0000_0000_0000, 0x3fc6_6666_6666_6666],
        spacing: [0x3ff0_0000_0000_0000, 0x4000_0000_0000_0000],
        png: false,
    },
];

const REFUSED: &[Refused] = &[
    Refused {
        what: "an elevation of three for two rows of two",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0.5 0.1" elevation="0 0 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        mujoco: "XML Error: elevation data length must match nrow*ncol",
        ours: "invalid elevation on <hfield>: elevation data length must match nrow*ncol (line 1: asset > hfield 'h')",
        png: false,
    },
    Refused {
        what: "no rows: the elevation not read, no data",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="0" ncol="3" size="1 1 0.5 0.1" elevation="0 0 0"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        mujoco: "Error: hfield not specified",
        ours: "hfield not specified (hfield 'h')",
        png: false,
    },
    Refused {
        what: "a negative row count",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="-1" ncol="3" size="1 1 0.5 0.1" elevation="0 0 0"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        mujoco: "Error: hfield not specified",
        ours: "hfield not specified (hfield 'h')",
        png: false,
    },
    Refused {
        what: "an elevation with no counts: not read",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" size="1 1 0.5 0.1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        mujoco: "Error: hfield not specified",
        ours: "hfield not specified (hfield 'h')",
        png: false,
    },
    Refused {
        what: "a top of 0",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0 0.1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        mujoco: "Error: size parameter is not positive in hfield",
        ours: "size parameter is not positive in hfield (hfield 'h')",
        png: false,
    },
    Refused {
        what: "a half-width of 0",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="2" ncol="2" size="0 1 1 1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/></worldbody></mujoco>"#,
        files: &[],
        mujoco: "Error: size parameter is not positive in hfield",
        ours: "size parameter is not positive in hfield (hfield 'h')",
        png: false,
    },
    Refused {
        what: "a negative half-width",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="2" ncol="2" size="1 -1 1 1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/></worldbody></mujoco>"#,
        files: &[],
        mujoco: "Error: size parameter is not positive in hfield",
        ours: "size parameter is not positive in hfield (hfield 'h')",
        png: false,
    },
    Refused {
        what: "a negative base",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="2" ncol="2" size="1 1 1 -1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/></worldbody></mujoco>"#,
        files: &[],
        mujoco: "Error: size parameter is not positive in hfield",
        ours: "size parameter is not positive in hfield (hfield 'h')",
        png: false,
    },
    Refused {
        what: "no rows beside an elevation that is not a number: no data",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="0" ncol="3" size="1 1 0.5 0.1" elevation="x y"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        mujoco: "Error: hfield not specified",
        ours: "hfield not specified (hfield 'h')",
        png: false,
    },
    Refused {
        what: "the first field with no data before the second's size",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" size="1 1 0.5 0.1"/><hfield name="h2" nrow="2" ncol="2" size="1 0 1 1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        mujoco: "Error: hfield not specified",
        ours: "hfield not specified (hfield 'h')",
        png: false,
    },
    Refused {
        what: "the first field's size before the second's data",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="2" ncol="2" size="1 0 1 1" elevation="0 0 1 1"/><hfield name="h2" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        mujoco: "Error: size parameter is not positive in hfield",
        ours: "size parameter is not positive in hfield (hfield 'h')",
        png: false,
    },
    Refused {
        what: "a field no geom uses, with no data",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0.5 0.1" elevation="0 0 1 1"/><hfield name="u" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        mujoco: "Error: hfield not specified",
        ours: "hfield not specified (hfield 'u')",
        png: false,
    },
    Refused {
        what: "a body's orientation before a field with no data",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0 0 1" zaxis="0 0 0"><joint type="hinge"/><geom size="0.1"/></body></worldbody></mujoco>"#,
        files: &[],
        mujoco: "Error: error 'zaxis too small' in frame alternative",
        ours: "invalid zaxis on <body>: zaxis too small (body #1)",
        png: false,
    },
    Refused {
        what: "a file named `.stl`",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="h.stl" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("h.stl", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x04\x00\x00\x00\x03\x08\x00\x00\x00\x00\x91\x9f\xf1\x1a\x00\x00\x00\x17IDATx\x9cc`ph\xf8\xcf\xc0%\x22\xa7\xc1p\x22\xc5H\x12\x00\x1b\xc9\x03\x9b\xdc\x00\x07\xa4\x00\x00\x00\x00IEND\xaeB`\x82")],
        mujoco: "Error: unsupported content type: 'model/stl'",
        ours: "unsupported content type: 'model/stl' (hfield 'h')",
        png: false,
    },
    Refused {
        what: "a content type MuJoCo does not accept: its own format, refused for the PNG's size",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="g8.png" content_type="foo" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("g8.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x04\x00\x00\x00\x03\x08\x00\x00\x00\x00\x91\x9f\xf1\x1a\x00\x00\x00\x17IDATx\x9cc`ph\xf8\xcf\xc0%\x22\xa7\xc1p\x22\xc5H\x12\x00\x1b\xc9\x03\x9b\xdc\x00\x07\xa4\x00\x00\x00\x00IEND\xaeB`\x82")],
        mujoco: "Error: unexpected file size in file '<dir>/g8.png'",
        ours: "unexpected file size in file '<dir>/g8.png' (hfield 'h')",
        png: false,
    },
    Refused {
        what: "a file not there",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="nowhere.png" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        mujoco: "Error: Error opening file 'nowhere.png'",
        ours: "asset file not found: 'nowhere.png' (resolved to '<dir>/nowhere.png') (hfield 'h')",
        png: true,
    },
    Refused {
        what: "MuJoCo's own format, every value NaN",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="allnan.bin" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("allnan.bin", b"\x01\x00\x00\x00\x02\x00\x00\x00\x00\x00\xc0\x7f\x00\x00\xc0\x7f")],
        mujoco: "Error: invalid data range in hfield 'allnan.bin'",
        ours: "invalid data range in hfield 'allnan.bin' (hfield 'h')",
        png: false,
    },
    Refused {
        what: "MuJoCo's own format, shorter than its counts",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="short.bin" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("short.bin", b"\x02\x00\x00\x00\x02")],
        mujoco: "Error: hfield missing header '<dir>/short.bin'",
        ours: "hfield missing header '<dir>/short.bin' (hfield 'h')",
        png: false,
    },
    Refused {
        what: "an empty file",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="empty.bin" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("empty.bin", b"")],
        mujoco: "Error: could not read hfield file '<dir>/empty.bin'",
        ours: "could not read hfield file '<dir>/empty.bin' (hfield 'h')",
        png: false,
    },
    Refused {
        what: "MuJoCo's own format, a negative count",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="negdims.bin" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("negdims.bin", b"\xfe\xff\xff\xff\x02\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00")],
        mujoco: "Error: non-positive hfield dimensions in file '<dir>/negdims.bin'",
        ours: "non-positive hfield dimensions in file '<dir>/negdims.bin' (hfield 'h')",
        png: false,
    },
    Refused {
        what: "MuJoCo's own format, too few values",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="badsize.bin" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("badsize.bin", b"\x02\x00\x00\x00\x02\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80?\x00\x00\x00@")],
        mujoco: "Error: unexpected file size in file '<dir>/badsize.bin'",
        ours: "unexpected file size in file '<dir>/badsize.bin' (hfield 'h')",
        png: false,
    },
    Refused {
        what: "content_type model/stl",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="g8.png" content_type="model/stl" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("g8.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x04\x00\x00\x00\x03\x08\x00\x00\x00\x00\x91\x9f\xf1\x1a\x00\x00\x00\x17IDATx\x9cc`ph\xf8\xcf\xc0%\x22\xa7\xc1p\x22\xc5H\x12\x00\x1b\xc9\x03\x9b\xdc\x00\x07\xa4\x00\x00\x00\x00IEND\xaeB`\x82")],
        mujoco: "Error: unsupported content type: 'model/stl'",
        ours: "unsupported content type: 'model/stl' (hfield 'h')",
        png: false,
    },
    Refused {
        what: "a written `file=\"\"`: no data",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        mujoco: "Error: hfield not specified",
        ours: "hfield not specified (hfield 'h')",
        png: false,
    },
    Refused {
        what: "a written `file=\"\"` beside counts and an elevation: neither read, no data",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="" nrow="2" ncol="2" elevation="0 1 2 3" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        mujoco: "Error: hfield not specified",
        ours: "hfield not specified (hfield 'h')",
        png: false,
    },
    Refused {
        what: "a file `strippath` leaves nothing of: no data",
        xml: r#"<mujoco model="h"><compiler strippath="true"/><option timestep="0.002"/><asset><hfield name="h" file="sub/" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        mujoco: "Error: hfield not specified",
        ours: "hfield not specified (hfield 'h')",
        png: false,
    },
    Refused {
        what: "counts whose product passes a C int",
        xml: r#"<mujoco model="h"><asset><hfield name="h" nrow="2147483647" ncol="2147483647" size="1 1 1 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/></worldbody></mujoco>"#,
        files: &[],
        mujoco: "Error: elevation data length must match nrow*ncol",
        ours: "elevation data length must match nrow*ncol (hfield 'h')",
        png: false,
    },
    Refused {
        what: "negative counts whose product passes a C int: not multiplied, no data",
        xml: r#"<mujoco><asset><hfield name="h" nrow="-65536" ncol="-65536" size="1 1 1 0.1"/></asset><worldbody><geom type="hfield" hfield="h"/></worldbody></mujoco>"#,
        files: &[],
        mujoco: "Error: hfield not specified",
        ours: "hfield not specified (hfield 'h')",
        png: false,
    },
    Refused {
        what: "the same with a top of 0: the size first",
        xml: r#"<mujoco><asset><hfield name="h" nrow="-65536" ncol="-65536" size="1 1 0 1"/></asset><worldbody><geom type="hfield" hfield="h"/></worldbody></mujoco>"#,
        files: &[],
        mujoco: "Error: size parameter is not positive in hfield",
        ours: "size parameter is not positive in hfield (hfield 'h')",
        png: false,
    },
    Refused {
        what: "a top of 0 and no data: the size first",
        xml: r#"<mujoco><asset><hfield name="h" size="1 1 0 0.1"/></asset><worldbody><geom type="hfield" hfield="h"/></worldbody></mujoco>"#,
        files: &[],
        mujoco: "Error: size parameter is not positive in hfield",
        ours: "size parameter is not positive in hfield (hfield 'h')",
        png: false,
    },
    Refused {
        what: "a top of 0 and a file not there: the size first",
        xml: r#"<mujoco><asset><hfield name="h" file="missing.bin" size="1 1 0 0.1"/></asset><worldbody><geom type="hfield" hfield="h"/></worldbody></mujoco>"#,
        files: &[],
        mujoco: "Error: size parameter is not positive in hfield",
        ours: "size parameter is not positive in hfield (hfield 'h')",
        png: false,
    },
    Refused {
        what: "a PNG whose header claims 2147483647 by 2147483647 pixels: refused before it is decoded",
        xml: r#"<mujoco><asset><hfield name="h" file="past_2_61.png" size="1 1 0.5 0.1"/></asset><worldbody><geom type="hfield" hfield="h"/></worldbody></mujoco>"#,
        files: &[("past_2_61.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x7f\xff\xff\xff\x7f\xff\xff\xff\x08\x00\x00\x00\x001\xa2T\xba\x00\x00\x00\x0aIDATx\x9cc`\x00\x00\x00\x02\x00\x01H\xaf\xa4q\x00\x00\x00\x00IEND\xaeB`\x82")],
        mujoco: "Error: error decoding PNG file '<dir>/past_2_61.png': integer overflow due to too many pixels",
        ours: "error decoding PNG file '<dir>/past_2_61.png': integer overflow due to too many pixels (hfield 'h')",
        png: true,
    },
    Refused {
        what: "MuJoCo's own format, every value NaN, the file named `./`: its reduced name",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="./nan_all.bin" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("nan_all.bin", b"\x02\x00\x00\x00\x02\x00\x00\x00\x00\x00\xc0\x7f\x00\x00\xc0\x7f\x00\x00\xc0\x7f\x00\x00\xc0\x7f")],
        mujoco: "Error: invalid data range in hfield 'nan_all.bin'",
        ours: "invalid data range in hfield 'nan_all.bin' (hfield 'h')",
        png: false,
    },
    Refused {
        what: "the same under `strippath`: its stripped name",
        xml: r#"<mujoco model="h"><compiler strippath="true"/><option timestep="0.002"/><asset><hfield name="h" file="sub/nan_all.bin" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("nan_all.bin", b"\x02\x00\x00\x00\x02\x00\x00\x00\x00\x00\xc0\x7f\x00\x00\xc0\x7f\x00\x00\xc0\x7f\x00\x00\xc0\x7f")],
        mujoco: "Error: invalid data range in hfield 'nan_all.bin'",
        ours: "invalid data range in hfield 'nan_all.bin' (hfield 'h')",
        png: false,
    },
    Refused {
        what: "the same through `sub/..`: its reduced name",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="sub/../nan_all.bin" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("nan_all.bin", b"\x02\x00\x00\x00\x02\x00\x00\x00\x00\x00\xc0\x7f\x00\x00\xc0\x7f\x00\x00\xc0\x7f\x00\x00\xc0\x7f")],
        mujoco: "Error: invalid data range in hfield 'nan_all.bin'",
        ours: "invalid data range in hfield 'nan_all.bin' (hfield 'h')",
        png: false,
    },
    Refused {
        what: "an empty PNG",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="empty.png" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("empty.png", b"")],
        mujoco: "Error: empty PNG file '<dir>/empty.png'",
        ours: "empty PNG file '<dir>/empty.png' (hfield 'h')",
        png: true,
    },
    Refused {
        what: "a file named `.obj`",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="h_obj.obj" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("h_obj.obj", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x04\x00\x00\x00\x03\x08\x00\x00\x00\x00\x91\x9f\xf1\x1a\x00\x00\x00\x17IDATx\x9cc`ph\xf8\xcf\xc0%\x22\xa7\xc1p\x22\xc5H\x12\x00\x1b\xc9\x03\x9b\xdc\x00\x07\xa4\x00\x00\x00\x00IEND\xaeB`\x82")],
        mujoco: "Error: unsupported content type: 'model/obj'",
        ours: "unsupported content type: 'model/obj' (hfield 'h')",
        png: false,
    },
    Refused {
        what: "a file named `.ply`",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="h_ply.ply" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("h_ply.ply", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x04\x00\x00\x00\x03\x08\x00\x00\x00\x00\x91\x9f\xf1\x1a\x00\x00\x00\x17IDATx\x9cc`ph\xf8\xcf\xc0%\x22\xa7\xc1p\x22\xc5H\x12\x00\x1b\xc9\x03\x9b\xdc\x00\x07\xa4\x00\x00\x00\x00IEND\xaeB`\x82")],
        mujoco: "Error: unsupported content type: 'model/ply'",
        ours: "unsupported content type: 'model/ply' (hfield 'h')",
        png: false,
    },
    Refused {
        what: "a file named `.msh`",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="h_msh.msh" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("h_msh.msh", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x04\x00\x00\x00\x03\x08\x00\x00\x00\x00\x91\x9f\xf1\x1a\x00\x00\x00\x17IDATx\x9cc`ph\xf8\xcf\xc0%\x22\xa7\xc1p\x22\xc5H\x12\x00\x1b\xc9\x03\x9b\xdc\x00\x07\xa4\x00\x00\x00\x00IEND\xaeB`\x82")],
        mujoco: "Error: unsupported content type: 'model/vnd.mujoco.msh'",
        ours: "unsupported content type: 'model/vnd.mujoco.msh' (hfield 'h')",
        png: false,
    },
    Refused {
        what: "MuJoCo's own format, a count of 0",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="zero_rows.bin" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("zero_rows.bin", b"\x00\x00\x00\x00\x02\x00\x00\x00")],
        mujoco: "Error: non-positive hfield dimensions in file '<dir>/zero_rows.bin'",
        ours: "non-positive hfield dimensions in file '<dir>/zero_rows.bin' (hfield 'h')",
        png: false,
    },
];

#[test]
fn each_height_field_is_mujocos() {
    let mut bad = Vec::new();
    for case in LOADS {
        if case.png && !cfg!(feature = "hfields") {
            continue;
        }
        let dir = tempfile::tempdir().unwrap();
        let model = match load(dir.path(), case.xml, case.files) {
            Ok(model) => model,
            Err(e) => {
                bad.push(format!("{}: refused: {e}", case.what));
                continue;
            }
        };
        let field = &model.hfield_data[0];
        let grid = (field.depth(), field.width());
        if grid != (case.nrow, case.ncol) {
            bad.push(format!(
                "{}: grid {grid:?}, MuJoCo's ({}, {})",
                case.what, case.nrow, case.ncol
            ));
            continue;
        }
        let heights: Vec<u64> = (0..case.nrow)
            .flat_map(|r| (0..case.ncol).map(move |c| (r, c)))
            .map(|(r, c)| field.get(c, r).unwrap().to_bits())
            .collect();
        if heights != case.heights {
            bad.push(format!(
                "{}: heights {heights:x?}, MuJoCo's {:x?}",
                case.what, case.heights
            ));
        }
        let geom = (0..model.ngeom)
            .find(|&g| model.geom_hfield[g].is_some())
            .unwrap();
        let size = model.geom_size[geom].map(f64::to_bits);
        if [size.x, size.y, size.z] != case.geom_size {
            bad.push(format!(
                "{}: geom size {:?}",
                case.what, model.geom_size[geom]
            ));
        }
        if field.spacing().map(f64::to_bits) != case.spacing {
            bad.push(format!("{}: spacing {:?}", case.what, field.spacing()));
        }
    }
    assert!(bad.is_empty(), "{}", bad.join("\n"));
}

#[test]
fn each_height_field_mujoco_refuses_is_refused() {
    let mut bad = Vec::new();
    for case in REFUSED {
        if case.png && !cfg!(feature = "hfields") {
            continue;
        }
        let dir = tempfile::tempdir().unwrap();
        // A path's separators read as `/`, so a Windows path reads alike.
        let slashes = |text: &str| text.replace('\\', "/");
        let want = slashes(
            &case
                .ours
                .replace("<dir>", &dir.path().display().to_string()),
        );
        match load(dir.path(), case.xml, case.files) {
            Ok(_) => bad.push(format!("{}: loads, MuJoCo: {}", case.what, case.mujoco)),
            Err(e) if slashes(&e.to_string()) == want => {}
            Err(e) => bad.push(format!("{}: {e}", case.what)),
        }
    }
    assert!(bad.is_empty(), "{}", bad.join("\n"));
}

/// Moving bodies on a field after 300 steps, MuJoCo 3.5.0's `qpos`.
struct Moving {
    what: &'static str,
    xml: &'static str,
    files: &'static [(&'static str, &'static [u8])],
    qpos: &'static [u64],
    png: bool,
}

const MOVING: &[Moving] = &[
    Moving {
        what: "a sphere falling onto two rows",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="2" ncol="2" size="1 1 0.5 0.1" elevation="0 0 1 1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        qpos: &[0x3fc0_a3d7_0a3d_70a4, 0x3fdc_3fba_ecc3_1ce2, 0x3fc8_6cd4_6da0_ccd4, 0xbfe7_3eb2_73b6_d612, 0xbfe5_fe1a_8fa4_0559, 0x3ce9_dccc_de1b_8131, 0xbcc2_591a_6f0d_2d5d],
        png: false,
    },
    Moving {
        what: "a sphere on a flat field moved to 0",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="4" ncol="4" size="1 1 0.5 0.1" elevation="0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        qpos: &[0x3fc0_a3d7_0a3d_70a4, 0x3fca_e147_ae14_7ae1, 0x3fa9_6982_8fff_c6c7, 0x3ff0_0000_0000_0000, 0x3d02_9e7a_e007_f11d, 0xbd0e_2ead_b714_1fb5, 0xb9cf_0384_c122_c168],
        png: false,
    },
    Moving {
        what: "a sphere on cells 0.5 by 0.5, a grid 9 by 5",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="5" ncol="9" size="2 1 0.2 0.1" elevation="0 0.7 0.3 1 0.6 0.2 0.9 0.5 0.1 0.8 0.4 0 0.7 0.3 1 0.6 0.2 0.9 0.5 0.1 0.8 0.4 0 0.7 0.3 1 0.6 0.2 0.9 0.5 0.1 0.8 0.4 0 0.7 0.3 1 0.6 0.2 0.9 0.5 0.1 0.8 0.4 0"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.3 -0.2 0.5"><freejoint/><geom type="sphere" size="0.06" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        qpos: &[0x3fd4_86ee_a9f3_710c, 0xbfc6_f222_ac19_1f53, 0x3fca_7e8f_6c73_7042, 0x3fef_0ed7_3a88_ef6b, 0xbfc5_ccad_83b8_d218, 0x3fc5_ccad_83b8_dd94, 0x3cfa_2738_2517_f4e8],
        png: false,
    },
    Moving {
        what: "a sphere, capsule, box and cylinder on a bumpy field, one in each quadrant",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="6" ncol="6" size="1 1 0.3 0.1" elevation="0 0.5 1 0.333333 0.833333 0.166667 0.833333 0.166667 0.666667 0 0.5 1 0.5 1 0.333333 0.833333 0.166667 0.666667 0.166667 0.666667 0 0.5 1 0.333333 1 0.333333 0.833333 0.166667 0.666667 0 0.666667 0 0.5 1 0.333333 0.833333"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.31 0.27 0.55"><freejoint/><geom type="sphere" size="0.07" mass="1"/></body><body pos="-0.42 0.33 0.55"><freejoint/><geom type="capsule" size="0.04 0.08" mass="1"/></body><body pos="-0.37 -0.41 0.55"><freejoint/><geom type="box" size="0.06 0.05 0.04" mass="1"/></body><body pos="0.44 -0.29 0.55"><freejoint/><geom type="cylinder" size="0.05 0.04" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        qpos: &[0x3fe3_ed5c_eefe_1ded, 0x3fc3_ceae_5235_352f, 0x3fc6_2783_7930_8af1, 0xbfec_bd77_d0ad_1c5c, 0x3fd4_e87d_128b_c480, 0x3fcb_348f_680e_d841, 0xbfca_108a_c4fb_8168, 0xbfd0_70f3_91d0_d684, 0x3fcf_c577_55f5_3051, 0x3fc7_26a0_9050_0c6a, 0x3fe0_20c0_c571_019e, 0x3fd4_4638_724f_ad3b, 0x3fe9_b60e_650c_4a51, 0xbf80_343f_016f_207e, 0xbfd6_b02a_b72a_4ec7, 0xbfd7_739d_8df8_b6d1, 0x3fc0_2aa1_ee9f_fa9a, 0x3fec_e244_431d_6494, 0xbfdb_0c99_cbe5_0655, 0xbf96_90e1_78f5_dbc2, 0x3fb4_18c1_9e91_787e, 0x3fd0_feb9_2076_6350, 0xbfdb_cafb_a7eb_ab0a, 0x3fc7_7a23_87f1_dfa7, 0xbfd0_fa75_98f2_ceed, 0x3fe5_4e0e_6b2c_73df, 0xbfe2_8dbe_2ff3_f1f7, 0x3fd8_cd88_f3a0_950c],
        png: false,
    },
    Moving {
        what: "a turned ellipsoid on the bumpy field",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="6" ncol="6" size="1 1 0.3 0.1" elevation="0 0.5 1 0.333333 0.833333 0.166667 0.833333 0.166667 0.666667 0 0.5 1 0.5 1 0.333333 0.833333 0.166667 0.666667 0.166667 0.666667 0 0.5 1 0.333333 1 0.333333 0.833333 0.166667 0.666667 0 0.666667 0 0.5 1 0.333333 0.833333"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="-0.2 -0.15 0.5" euler="20 10 5"><freejoint/><geom type="ellipsoid" size="0.07 0.05 0.04" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        qpos: &[0xbfca_7bac_7323_0d2d, 0xbfc8_7227_4ca8_687c, 0x3faa_62df_931c_fb0c, 0x3fec_2f46_2ced_f129, 0x3fc6_802a_1e87_6237, 0x3fb7_7d08_dbb5_ebe2, 0x3fdb_8592_3732_cfac],
        png: false,
    },
    Moving {
        what: "a tilted capsule on the bumpy field",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="6" ncol="6" size="1 1 0.3 0.1" elevation="0 0.5 1 0.333333 0.833333 0.166667 0.833333 0.166667 0.666667 0 0.5 1 0.5 1 0.333333 0.833333 0.166667 0.666667 0.166667 0.666667 0 0.5 1 0.333333 1 0.333333 0.833333 0.166667 0.666667 0 0.666667 0 0.5 1 0.333333 0.833333"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="-0.5 0.1 0.5" euler="0 60 30"><freejoint/><geom type="capsule" size="0.03 0.12" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        qpos: &[0xbfdb_2674_8547_7371, 0xbfa9_1a4d_2807_eb3b, 0x3fc7_8986_c301_11e2, 0xbfe0_e183_22a4_e9d1, 0x3fcd_3e3f_b68c_00cf, 0xbfea_0527_acb0_c6a1, 0x3fb7_60dc_8796_e6ce],
        png: false,
    },
    Moving {
        what: "a sphere on the bumpy field turned and moved",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="6" ncol="6" size="1 1 0.3 0.1" elevation="0 0.5 1 0.333333 0.833333 0.166667 0.833333 0.166667 0.666667 0 0.5 1 0.5 1 0.333333 0.833333 0.166667 0.666667 0.166667 0.666667 0 0.5 1 0.333333 1 0.333333 0.833333 0.166667 0.666667 0 0.666667 0 0.5 1 0.333333 0.833333"/></asset><worldbody><geom name="hf" type="hfield" hfield="h" pos="0.2 -0.1 0.05" euler="0 0 30"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[],
        qpos: &[0xbfb2_ad2c_5ab1_0b9d, 0x3fda_d107_4880_aeb3, 0x3fc8_8ec4_48a7_6bbd, 0xbfe6_6946_c9c0_dd91, 0x3fe1_9e2a_37d4_36af, 0x3fcc_8f31_4f09_0f94, 0x3fd9_53da_7487_5ec5],
        png: false,
    },
    Moving {
        what: "a sphere and a cylinder on a PNG's field",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="g8.png" size="1 0.8 0.3 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.4 0.3 0.5"><freejoint/><geom type="sphere" size="0.06" mass="1"/></body><body pos="-0.4 -0.3 0.5"><freejoint/><geom type="cylinder" size="0.05 0.03" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("g8.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x04\x00\x00\x00\x03\x08\x00\x00\x00\x00\x91\x9f\xf1\x1a\x00\x00\x00\x17IDATx\x9cc`ph\xf8\xcf\xc0%\x22\xa7\xc1p\x22\xc5H\x12\x00\x1b\xc9\x03\x9b\xdc\x00\x07\xa4\x00\x00\x00\x00IEND\xaeB`\x82")],
        qpos: &[0x3fcb_48f0_8a8c_3638, 0x3fc4_ae8a_b236_7319, 0x3fbd_e166_5a46_3aa4, 0xbfd8_c3da_f58d_47be, 0x3fe1_318c_2c08_f783, 0xbfe7_cbeb_50ad_c710, 0xbfb7_b400_17f9_deb2, 0xbfd8_6fb6_3e50_3c61, 0xbfd2_70d9_1148_d53c, 0x3fb7_6dea_cdd5_fdfb, 0x3fef_cfd8_242a_db09, 0xbfae_7ccd_9c5f_c596, 0x3fb6_b78d_98d3_f4c9, 0xbf91_cb9c_3728_51e4],
        png: true,
    },
    Moving {
        what: "a sphere on an 8-bit grey PNG's field",
        xml: r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" file="g8.png" size="1 1 0.5 0.1"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0.13 0.21 0.6"><freejoint/><geom type="sphere" size="0.05" mass="1"/></body></worldbody></mujoco>"#,
        files: &[("g8.png", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x04\x00\x00\x00\x03\x08\x00\x00\x00\x00\x91\x9f\xf1\x1a\x00\x00\x00\x17IDATx\x9cc`ph\xf8\xcf\xc0%\x22\xa7\xc1p\x22\xc5H\x12\x00\x1b\xc9\x03\x9b\xdc\x00\x07\xa4\x00\x00\x00\x00IEND\xaeB`\x82")],
        qpos: &[0x3fba_a9fb_d384_8051, 0x3fa5_1226_e6eb_19fd, 0x3fbc_4d88_6afc_9126, 0xbfc6_30cc_7619_853d, 0x3fef_2713_a87e_07c1, 0xbfc3_12b3_0cb6_812b, 0x3c9a_e57f_d427_dc4c],
        png: true,
    },
];

/// Three rangefinders over a field whose rows differ read from either end:
/// MuJoCo 3.5.0's readings after `mj_forward`.
const RANGEFINDERS: &str = r#"<mujoco model="h"><option timestep="0.002"/><asset><hfield name="h" nrow="3" ncol="3" size="1 1 0.5 0.1" elevation="0 0 0 0.2 0.5 0.9 1 0.1 0.3"/></asset><worldbody><geom name="hf" type="hfield" hfield="h"/><body pos="0 0 1"><site name="s1" pos="0.3 0.6 0" euler="180 0 0"/><site name="s2" pos="-0.7 -0.4 0" euler="180 0 0"/><site name="s3" pos="0.1 -0.8 0" euler="180 0 0"/></body></worldbody><sensor><rangefinder site="s1"/><rangefinder site="s2"/><rangefinder site="s3"/></sensor></mujoco>"#;
const READINGS: [u64; 3] = [
    0x3fec_cccc_cccc_cccd,
    0x3fe6_3d70_a399_999a,
    0x3fec_7ae1_4800_0000,
];

/// A run's values against MuJoCo's: bit for bit on macOS arm64, where the
/// oracle ran; elsewhere within 1e-9 of the array's largest entry, as
/// another libm moves the last bits. A `NaN` fails either way.
fn close(ours: &[f64], mujoco: &[u64]) -> bool {
    let want: Vec<f64> = mujoco.iter().map(|&b| f64::from_bits(b)).collect();
    if ours.len() != want.len() || ours.iter().chain(&want).any(|v| v.is_nan()) {
        return false;
    }
    if cfg!(all(target_os = "macos", target_arch = "aarch64")) {
        return ours.iter().map(|v| v.to_bits()).eq(mujoco.iter().copied());
    }
    let scale = want.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
    ours.iter()
        .zip(&want)
        .all(|(a, b)| (a - b).abs() <= 1e-9 * scale)
}

#[test]
fn bodies_on_height_fields_move_as_mujocos() {
    let mut bad = Vec::new();
    for case in MOVING {
        if case.png && !cfg!(feature = "hfields") {
            continue;
        }
        let dir = tempfile::tempdir().unwrap();
        let model = load(dir.path(), case.xml, case.files).unwrap();
        let mut data = model.make_data();
        for _ in 0..300 {
            data.step(&model).unwrap();
        }
        if !close(data.qpos.as_slice(), case.qpos) {
            bad.push(format!("{}: qpos {:?}", case.what, data.qpos.as_slice()));
        }
    }
    assert!(bad.is_empty(), "{}", bad.join("\n"));
}

#[test]
fn rangefinders_read_a_height_field_as_mujocos() {
    let dir = tempfile::tempdir().unwrap();
    let model = load(dir.path(), RANGEFINDERS, &[]).unwrap();
    let mut data = model.make_data();
    data.forward(&model).unwrap();
    assert!(
        close(data.sensordata.as_slice(), &READINGS),
        "{:?}",
        data.sensordata.as_slice()
    );
}

/// Registry `D-HFIELD-COUNTS`: a grid of more cells than a C `int` counts,
/// where MuJoCo 3.5.0's `int` products overflow (each measured on the
/// oracle), refused before anything is allocated: positive counts with no
/// data, in the words MuJoCo gives the largest such grid, (2³¹ − 1)²
/// (65536 × 65536 crashes it, 46341 × 46341 and (2³¹ − 1) × 2 allocate past
/// 2.5 GB); a PNG whose header claims one, in lodepng's words for an image
/// past 2⁶¹ pixels (for 46341 × 46341 pixels and one row of data, MuJoCo's
/// are "invalid decompressed idat size"). MuJoCo's own format's file, which
/// this crate sizes in 64 bits, is refused where it is shorter than its
/// header's counts give, "unexpected file size" (MuJoCo's: "could not
/// allocate buffers in hfield" for counts 65536 × 65536 and no data,
/// "nhfielddata is too large" for (2³¹ − 1)² and one value).
#[test]
fn a_grid_past_an_int_is_refused() {
    for (nrow, ncol) in [(65536, 65536), (46341, 46341), (2_147_483_647, 2)] {
        let xml = format!(
            r#"<mujoco><asset><hfield name="h" nrow="{nrow}" ncol="{ncol}" size="1 1 1 0.1"/></asset><worldbody><geom type="hfield" hfield="h"/></worldbody></mujoco>"#
        );
        let error = sim_mjcf::load_model(&xml).err().map(|e| e.to_string());
        assert_eq!(
            error.as_deref(),
            Some("elevation data length must match nrow*ncol (hfield 'h')"),
            "{nrow} x {ncol}"
        );
    }
    let dir = tempfile::tempdir().unwrap();
    let mut files: Vec<(&str, &[u8])> =
        vec![("rows.bin", HEADER_65536), ("most.bin", HEADER_INT_MAX)];
    if cfg!(feature = "hfields") {
        files.push(("pixels.png", PNG_PAST_AN_INT));
    }
    for (file, bytes) in files {
        let xml = format!(
            r#"<mujoco><asset><hfield name="h" file="{file}" size="1 1 1 0.1"/></asset><worldbody><geom type="hfield" hfield="h"/></worldbody></mujoco>"#
        );
        let error = load(dir.path(), &xml, &[(file, bytes)]).err();
        let path = dir.path().join(file).display().to_string();
        let want = if file == "pixels.png" {
            format!(
                "error decoding PNG file '{path}': integer overflow due to too many pixels (hfield 'h')"
            )
        } else {
            format!("unexpected file size in file '{path}' (hfield 'h')")
        };
        assert_eq!(
            error.map(|e| e.to_string().replace('\\', "/")),
            Some(want.replace('\\', "/")),
            "{file}"
        );
    }
}

/// MuJoCo's own format: counts 65536 and 65536, and no values.
const HEADER_65536: &[u8] = b"\x00\x00\x01\x00\x00\x00\x01\x00";

/// MuJoCo's own format: counts 2147483647 and 2147483647, and one value.
const HEADER_INT_MAX: &[u8] = b"\xff\xff\xff\x7f\xff\xff\xff\x7f\x00\x00\x00?";

/// An 8-bit grey PNG whose header claims 46341 by 46341 pixels, its data one
/// row.
const PNG_PAST_AN_INT: &[u8] = b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\xb5\x05\x00\x00\xb5\x05\x08\x00\x00\x00\x00\xf4\x95\xcb\xff\x00\x00\x00\x0aIDATx\x9cc`\x00\x00\x00\x02\x00\x01H\xaf\xa4q\x00\x00\x00\x00IEND\xaeB`\x82";

/// Registry `D-HFIELD-PNG`: a malformed PNG is decided by the `png` crate,
/// not by lodepng, as MuJoCo 3.5.0 decides it (each measured on the oracle;
/// review round 1 RA's files): a palette whose length is not a multiple of
/// 3, which lodepng reads to its last whole entry and the `png` crate panics
/// on, is refused; a stream whose Adler-32 checksum is wrong, which lodepng
/// refuses, loads; an image whose decoding needs more memory than this
/// crate's bound (a 16-bit RGBA header of 46340 by 46340 pixels, 17 GB
/// decoded, its data one row), which lodepng refuses as its data is short
/// ("invalid decompressed idat size"), is refused before anything is
/// allocated, "Memory limit exceeded"; one below the bound and above the
/// image crate's own bound of 2²⁹ bytes (an RGBA16 header of 8193 by 8192
/// pixels, 537 MB decoded, its data one row) is decoded and refused for its
/// short data, as MuJoCo loads the whole such image (measured, 1.7 GB); a
/// header past an int whose IHDR CRC is wrong is refused for its CRC, as
/// lodepng checks the header before the pixels, and so is each other
/// header past an int lodepng refuses (its signature, its first chunk's
/// length or name, its colour type or bit depth, its compression, filter or
/// interlace method), in the decoder's words for its fault.
#[cfg(feature = "hfields")]
#[test]
fn a_malformed_png_is_decided_by_this_crates_decoder() {
    let xml = |file: &str| {
        format!(
            r#"<mujoco><asset><hfield name="h" file="{file}" size="1 1 0.5 0.1"/></asset><worldbody><geom type="hfield" hfield="h"/></worldbody></mujoco>"#
        )
    };
    let dir = tempfile::tempdir().unwrap();
    let palette = load(
        dir.path(),
        &xml("palette.png"),
        &[("palette.png", PALETTE_NOT_IN_THREES)],
    );
    let want = format!(
        "error decoding PNG file '{}': the decoder failed (hfield 'h')",
        dir.path().join("palette.png").display()
    );
    assert_eq!(
        palette.err().map(|e| e.to_string().replace('\\', "/")),
        Some(want.replace('\\', "/"))
    );
    let dir = tempfile::tempdir().unwrap();
    let adler = load(dir.path(), &xml("adler.png"), &[("adler.png", BAD_ADLER32)]);
    assert!(adler.is_ok(), "{:?}", adler.err());
    let large = load(
        dir.path(),
        &xml("large.png"),
        &[("large.png", PNG_PAST_THE_LIMIT)],
    );
    let want = format!(
        "error decoding PNG file '{}': Memory limit exceeded (hfield 'h')",
        dir.path().join("large.png").display()
    );
    assert_eq!(
        large.err().map(|e| e.to_string().replace('\\', "/")),
        Some(want.replace('\\', "/"))
    );
    let below = load(
        dir.path(),
        &xml("below.png"),
        &[("below.png", PNG_UNDER_THE_BOUND)],
    );
    let below = below.err().map(|e| e.to_string()).unwrap_or_default();
    assert!(
        below.contains("does not have enough data") && !below.contains("Memory limit"),
        "{below}"
    );
    let crc = load(
        dir.path(),
        &xml("crc.png"),
        &[("crc.png", PNG_BAD_CRC_PAST_AN_INT)],
    );
    let crc = crc.err().map(|e| e.to_string()).unwrap_or_default();
    assert!(
        crc.contains("CRC error") && !crc.contains("too many pixels"),
        "{crc}"
    );
    for (what, bytes) in PNG_HEADERS_LODEPNG_REFUSES {
        let refused = load(dir.path(), &xml("header.png"), &[("header.png", bytes)]);
        let refused = refused.err().map(|e| e.to_string()).unwrap_or_default();
        assert!(
            refused.contains("error decoding PNG file") && !refused.contains("too many pixels"),
            "{what}: {refused}"
        );
    }
}

/// A palette PNG 4 pixels wide and 3 high whose `PLTE` chunk is 13 bytes
/// long.
#[cfg(feature = "hfields")]
const PALETTE_NOT_IN_THREES: &[u8] = b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x04\x00\x00\x00\x03\x08\x03\x00\x00\x00\x83*^\xf4\x00\x00\x00\x0dPLTE\x0a\xc8\x1eZPF\xfa\x01\x02\x00\x00\x00\x07\x87L\xfb\xa2\x00\x00\x00\x17IDATx\x9cc``dbf`fbd``ddb\x02\x00\x00\x94\x00\x13$\xc5\xffa\x00\x00\x00\x00IEND\xaeB`\x82";

/// A grey PNG 4 pixels wide and 3 high whose zlib stream's Adler-32
/// checksum is wrong.
#[cfg(feature = "hfields")]
const BAD_ADLER32: &[u8] = b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\x00\x04\x00\x00\x00\x03\x08\x00\x00\x00\x00\x91\x9f\xf1\x1a\x00\x00\x00\x17IDATx\x9cc`ph\xf8\xcf\xc0%\x22\xa7\xc1p\x22\xc5H\x12\x00\xe4\xc9\x03\x9b\x02\x05\x01U\x00\x00\x00\x00IEND\xaeB`\x82";

/// A 16-bit RGBA PNG whose header claims 46340 by 46340 pixels, its data one
/// row.
#[cfg(feature = "hfields")]
const PNG_PAST_THE_LIMIT: &[u8] = b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00\xb5\x04\x00\x00\xb5\x04\x10\x06\x00\x00\x00\xa5\xf0\xf0\xfb\x00\x00\x00\x0aIDATx\x9cc`\x00\x00\x00\x02\x00\x01H\xaf\xa4q\x00\x00\x00\x00IEND\xaeB`\x82";

/// A 16-bit RGBA PNG whose header claims 8193 by 8192 pixels, its data one
/// row.
#[cfg(feature = "hfields")]
const PNG_UNDER_THE_BOUND: &[u8] = b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x00\x00 \x01\x00\x00 \x00\x10\x06\x00\x00\x00\xcd\xf8}$\x00\x00\x00VIDATx\xda\xed\xc11\x01\x00\x00\x00\xc2\xa0\xf5O\xedk\x08\xa0\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xe0\x0c\x00\x18\x00\x01\x8f\xe0\xb0D\x00\x00\x00\x00IEND\xaeB`\x82";

/// An 8-bit grey PNG whose header claims 2147483647 by 2147483647 pixels,
/// its IHDR's CRC 0, its data one row.
#[cfg(feature = "hfields")]
const PNG_BAD_CRC_PAST_AN_INT: &[u8] = b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x7f\xff\xff\xff\x7f\xff\xff\xff\x08\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x0aIDATx\x9cc`\x00\x00\x00\x02\x00\x01H\xaf\xa4q\x00\x00\x00\x00IEND\xaeB`\x82";

/// 8-bit grey PNGs whose header claims 2147483647 by 2147483647 pixels and
/// holds one fault lodepng refuses, each beside what it is; their data one
/// row.
#[cfg(feature = "hfields")]
const PNG_HEADERS_LODEPNG_REFUSES: &[(&str, &[u8])] = &[
    ("a wrong signature", b"\x88PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x7f\xff\xff\xff\x7f\xff\xff\xff\x08\x00\x00\x00\x001\xa2T\xba\x00\x00\x00\x0aIDATx\x9cc`\x00\x00\x00\x02\x00\x01H\xaf\xa4q\x00\x00\x00\x00IEND\xaeB`\x82"),
    ("a first chunk 14 bytes long, its CRC right over the 13 lodepng reads", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0eIHDR\x7f\xff\xff\xff\x7f\xff\xff\xff\x08\x00\x00\x00\x001\xa2T\xba\x00\x00\x00\x0aIDATx\x9cc`\x00\x00\x00\x02\x00\x01H\xaf\xa4q\x00\x00\x00\x00IEND\xaeB`\x82"),
    ("a first chunk not named IHDR", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDX\x7f\xff\xff\xff\x7f\xff\xff\xff\x08\x00\x00\x00\x00\xe3\x95\x8e!\x00\x00\x00\x0aIDATx\x9cc`\x00\x00\x00\x02\x00\x01H\xaf\xa4q\x00\x00\x00\x00IEND\xaeB`\x82"),
    ("colour type 5", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x7f\xff\xff\xff\x7f\xff\xff\xff\x08\x05\x00\x00\x00\x06|\xa4\x88\x00\x00\x00\x0aIDATx\x9cc`\x00\x00\x00\x02\x00\x01H\xaf\xa4q\x00\x00\x00\x00IEND\xaeB`\x82"),
    ("a grey of 3 bits", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x7f\xff\xff\xff\x7f\xff\xff\xff\x03\x00\x00\x00\x00Fre\xab\x00\x00\x00\x0aIDATx\x9cc`\x00\x00\x00\x02\x00\x01H\xaf\xa4q\x00\x00\x00\x00IEND\xaeB`\x82"),
    ("compression method 1", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x7f\xff\xff\xff\x7f\xff\xff\xff\x08\x00\x01\x00\x000`>\x8d\x00\x00\x00\x0aIDATx\x9cc`\x00\x00\x00\x02\x00\x01H\xaf\xa4q\x00\x00\x00\x00IEND\xaeB`\x82"),
    ("filter method 1", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x7f\xff\xff\xff\x7f\xff\xff\xff\x08\x00\x00\x01\x00(\xb9e\xfb\x00\x00\x00\x0aIDATx\x9cc`\x00\x00\x00\x02\x00\x01H\xaf\xa4q\x00\x00\x00\x00IEND\xaeB`\x82"),
    ("interlace method 2", b"\x89PNG\x0d\x0a\x1a\x0a\x00\x00\x00\x0dIHDR\x7f\xff\xff\xff\x7f\xff\xff\xff\x08\x00\x00\x00\x02\xdf\xac5\x96\x00\x00\x00\x0aIDATx\x9cc`\x00\x00\x00\x02\x00\x01H\xaf\xa4q\x00\x00\x00\x00IEND\xaeB`\x82"),
];
