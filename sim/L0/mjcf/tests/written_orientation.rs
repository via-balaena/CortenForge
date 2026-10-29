//! A written orientation compiled with MuJoCo 3.5.0's arithmetic, and the
//! alternatives it refuses (book L14; ledger L228, L264).
//!
//! `CASES` are MuJoCo 3.5.0's `body_quat`, `geom_quat` and `site_quat`, as bits,
//! measured on the census's unfused oracle; MuJoCo gives the three elements the
//! same quaternion for each. Its compiler makes a written quaternion unit with
//! `mjuu_normvec`, which keeps one within mjEPS of unit length as written
//! (`user_util.cc:149-170`; `mjCBody::Compile`, `user_objects.cc:2388`;
//! `mjCGeom::Compile`, `:3675`; a site after its frame, `:3988`), and resolves an
//! alternative with `ResolveOrientation` (`:239-346`), which refuses an axis too
//! small to make unit. `quat` and `xyaxes` take only arithmetic and square roots,
//! so they are compared bit for bit everywhere; `euler`, `axisangle` and `zaxis`
//! pass through `sin`, `cos` and `atan2`, whose last bit another platform's libm
//! may round otherwise, so off macOS arm64 they are compared within 1e-14.
//!
//! A frame composes its turn with nalgebra's arithmetic, not
//! `mjuu_frameaccumChild` (ledger L255), so `FRAME_CASES` are compared within
//! 1e-14 only.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
// The cases are bit patterns, written as MuJoCo's values' raw bits.
#![allow(clippy::unreadable_literal)]

use sim_mjcf::{MjcfError, load_model};

/// `(attribute, compiler element, MuJoCo's quaternion as bits, passes through libm)`.
const CASES: &[(&str, &str, [u64; 4], bool)] = &[
    // quat_L228
    (
        r#"quat="0.98 0.05 0.1 0.15""#,
        "",
        [
            0x3fef6eb04fbbb787,
            0x3fa9a8b9b9406c06,
            0x3fb9a8b9b9406c06,
            0x3fc33e8b4af05104,
        ],
        false,
    ),
    // quat_unit_L228
    (
        r#"quat="0.9621023987294832 0.10127393670836667 0.20254787341673333 0.15191090506254998""#,
        "",
        [
            0x3feec98af83e47f2,
            0x3fb9ed16b6197ff5,
            0x3fc9ed16b6197ff5,
            0x3fc371d108931ff7,
        ],
        false,
    ),
    // quat_L264
    (
        r#"quat="0 0.7071067811865489 0 0.7071067811865489""#,
        "",
        [
            0x0000000000000000,
            0x3fe6a09e667f3bd9,
            0x0000000000000000,
            0x3fe6a09e667f3bd9,
        ],
        false,
    ),
    // quat_1234
    (
        r#"quat="1 2 3 4""#,
        "",
        [
            0x3fc75e9746a0b098,
            0x3fd75e9746a0b098,
            0x3fe186f174f88472,
            0x3fe75e9746a0b098,
        ],
        false,
    ),
    // euler_deg
    (
        r#"euler="30 20 10""#,
        "",
        [
            0x3fee32e87758ecc0,
            0x3fd12fb0e0216fb2,
            0x3fc28b5dcd4964be,
            0x3fc057ccc7147938,
        ],
        true,
    ),
    // euler_deg_b
    (
        r#"euler="-37 112 7.5""#,
        "",
        [
            0x3fe17bd2f2d3fdc6,
            0xbfc014ce3ba66b8c,
            0x3fe979cb36a9d0a7,
            0xbfcd28eb40691dde,
        ],
        true,
    ),
    // euler_rad_ZYX
    (
        r#"euler="0.3 0.2 0.1""#,
        r#"<compiler angle="radian" eulerseq="ZYX"/>"#,
        [
            0x3fef6b5da2fabcbf,
            0x3fb066fad7021ab9,
            0x3fb75619e5362ec9,
            0x3fc3a3e6280ebbaf,
        ],
        true,
    ),
    // euler_XYz
    (
        r#"euler="10 20 30""#,
        r#"<compiler eulerseq="XYz"/>"#,
        [
            0x3fee7315e43cde2e,
            0x3fc057ccc7147938,
            0x3fc28b5dcd4964bd,
            0x3fcea153f34ce654,
        ],
        true,
    ),
    // axisangle_deg
    (
        r#"axisangle="1 2 3 45""#,
        "",
        [
            0x3fed906bcf328d46,
            0x3fba2eca154b7872,
            0x3fca2eca154b7872,
            0x3fd3a3178ff89a56,
        ],
        true,
    ),
    // axisangle_rad
    (
        r#"axisangle="0.3 -0.7 1.1 2.5""#,
        r#"<compiler angle="radian"/>"#,
        [
            0x3fd42e3dd88bd952,
            0x3fcb3cbefca37185,
            0xbfdfc6ded16959c7,
            0x3fe8f7af12407d66,
        ],
        true,
    ),
    // xyaxes
    (
        r#"xyaxes="1 2 3 -1 0.5 0.2""#,
        "",
        [
            0x3fe7e48c524fe2e4,
            0x3fd1a917c4b8acd6,
            0xbfd6c8cad9b3f76b,
            0x3fdf544a7119f30c,
        ],
        false,
    ),
    // xyaxes_neg_trace
    (
        r#"xyaxes="-1 0.1 0 0 -1 0.2""#,
        "",
        [
            0x3fa96118ceff5ab0,
            0x3f7433e1e7b684c5,
            0x3fb950f99851c053,
            0x3fefcd9fc2359d71,
        ],
        false,
    ),
    // xyaxes_q2
    (
        r#"xyaxes="-1 0 0.1 0 1 0""#,
        "",
        [
            0xbfa9812e78ef9b10,
            0x0000000000000000,
            0x3feff5d47137c7dd,
            0x0000000000000000,
        ],
        false,
    ),
    // xyaxes_q3
    (
        r#"xyaxes="-1 0.1 0 0.1 1 0""#,
        "",
        [
            0x0000000000000000,
            0x3fa9812e78ef9b10,
            0x3feff5d47137c7de,
            0x0000000000000000,
        ],
        false,
    ),
    // zaxis
    (
        r#"zaxis="0.3 -0.7 1.1""#,
        "",
        [
            0x3fee8b5883388cd1,
            0x3fd18a5bb8a5ebc6,
            0x3fbe11c1ced34b09,
            0x8000000000000000,
        ],
        true,
    ),
    // zaxis_down
    (
        r#"zaxis="0 0 -1""#,
        "",
        [
            0x3c91a62633145c07,
            0x3ff0000000000000,
            0x0000000000000000,
            0x0000000000000000,
        ],
        true,
    ),
    // zaxis_just_enough
    (
        r#"zaxis="2e-7 0 0""#,
        "",
        [
            0x3fe6a09e667f3bcd,
            0x0000000000000000,
            0x3fe6a09e667f3bcc,
            0x0000000000000000,
        ],
        true,
    ),
];

/// `(attribute, the attribute MuJoCo names, MuJoCo's reason)`: refused on a body,
/// a geom, a site and a frame alike.
const REFUSED: &[(&str, &str, &str)] = &[
    (
        r#"axisangle="0 0 0 30""#,
        "axisangle",
        "axisangle too small",
    ),
    (
        r#"axisangle="1e-8 0 0 30""#,
        "axisangle",
        "axisangle too small",
    ),
    (r#"xyaxes="0 0 0 0 1 0""#, "xyaxes", "xaxis too small"),
    (r#"xyaxes="1 0 0 2 0 0""#, "xyaxes", "yaxis too small"),
    (r#"zaxis="0 0 0""#, "zaxis", "zaxis too small"),
    (r#"zaxis="1e-8 0 0""#, "zaxis", "zaxis too small"),
    // An axis whose sum of squares overflows: `normvec` makes it 0 (its norm
    // infinite), so the cross product is 0.
    (
        r#"xyaxes="1e200 0 0 0 1e200 0""#,
        "xyaxes",
        "cross(xaxis, yaxis) too small",
    ),
    (
        r#"xyaxes="1 0 0 0 1e160 0""#,
        "xyaxes",
        "cross(xaxis, yaxis) too small",
    ),
    (
        r#"xyaxes="1e160 0 0 0 1 0""#,
        "xyaxes",
        "cross(xaxis, yaxis) too small",
    ),
];

#[derive(Clone, Copy, Debug)]
enum Element {
    Body,
    Geom,
    Site,
}

fn document(element: Element, attr: &str, compiler: &str) -> String {
    let inner = match element {
        Element::Body => format!(
            r#"<body name="b" {attr}><joint type="hinge"/><geom type="sphere" size="0.1"/></body>"#
        ),
        Element::Geom => format!(
            r#"<body name="b"><joint type="hinge"/><geom type="box" size="0.1 0.2 0.3" {attr}/></body>"#
        ),
        Element::Site => format!(
            r#"<body name="b"><joint type="hinge"/><geom type="sphere" size="0.1"/><site name="s" {attr}/></body>"#
        ),
    };
    format!("<mujoco>{compiler}<worldbody>{inner}</worldbody></mujoco>")
}

fn quat_of(element: Element, attr: &str, compiler: &str) -> [f64; 4] {
    let model = load_model(&document(element, attr, compiler))
        .unwrap_or_else(|e| panic!("{element:?} {attr}: {e}"));
    let q = match element {
        Element::Body => model.body_quat[1],
        Element::Geom => model.geom_quat[0],
        Element::Site => model.site_quat[0],
    }
    .into_inner();
    [q.w, q.i, q.j, q.k]
}

/// Whether `got` is `mujoco` bit for bit, or within `tol` per component where
/// `tol` is given (a `NaN` fails).
fn is_mujocos(got: [f64; 4], mujoco: [f64; 4], tol: Option<f64>) -> bool {
    got.iter()
        .zip(&mujoco)
        .all(|(a, b)| tol.map_or_else(|| a.to_bits() == b.to_bits(), |tol| (a - b).abs() <= tol))
}

#[test]
fn a_written_orientation_is_mujocos_on_a_body_a_geom_and_a_site() {
    let here = cfg!(all(target_os = "macos", target_arch = "aarch64"));
    let mut differ = Vec::new();
    for &(attr, compiler, bits, libm) in CASES {
        let mujoco = bits.map(f64::from_bits);
        let tol = (libm && !here).then_some(1e-14);
        for element in [Element::Body, Element::Geom, Element::Site] {
            let got = quat_of(element, attr, compiler);
            if !is_mujocos(got, mujoco, tol) {
                differ.push(format!(
                    "{element:?} {attr} {compiler}: {got:?}, MuJoCo {mujoco:?}"
                ));
            }
        }
    }
    assert!(
        differ.is_empty(),
        "{} of {} differ:\n  {}",
        differ.len(),
        3 * CASES.len(),
        differ.join("\n  ")
    );
}

#[test]
fn an_alternative_too_small_to_make_unit_is_refused_as_mujoco_refuses_it() {
    for &(attr, attribute, reason) in REFUSED {
        for (element, tag) in [
            (Element::Body, "body"),
            (Element::Geom, "geom"),
            (Element::Site, "site"),
        ] {
            let err = load_model(&document(element, attr, "")).unwrap_err();
            assert!(
                matches!(&err, MjcfError::InvalidValue { element: e, attribute: a, reason: r, .. }
                    if e == tag && a == attribute && r == reason),
                "{tag} {attr}: {err}"
            );
        }
        let in_frame = format!(
            r#"<mujoco><worldbody><body name="b"><joint type="hinge"/><geom type="sphere" size="0.1"/><frame {attr}><geom type="box" size="0.1 0.2 0.3"/></frame></body></worldbody></mujoco>"#
        );
        let err = load_model(&in_frame).unwrap_err();
        assert!(
            matches!(&err, MjcfError::InvalidValue { element: e, attribute: a, reason: r, .. }
                if e == "frame" && a == attribute && r == reason),
            "frame {attr}: {err}"
        );
    }
}

/// 120 orientations drawn with a seeded generator (Python's `random.Random(14)`,
/// six decimals): 40 `euler` over eight sequences, half in radians, 30
/// `axisangle`, half in radians, 20 `zaxis`, 20 `xyaxes` and 10 unnormalised
/// `quat`; MuJoCo 3.5.0's `body_quat` for each, as bits. On a body only: the
/// resolver is the one every element uses.
const SEEDED: &[(&str, &str, [u64; 4], bool)] = &[
    (
        r#"euler="-2.516297 1.296547 0.973069""#,
        r#"<compiler angle="radian" eulerseq="xyz"/>"#,
        [
            0x3fdf1032d74cf4fc,
            0xbfe2ad1eef76158c,
            0x3fe09a16c46c3da3,
            0xbfd92ab5975e3eed,
        ],
        true,
    ),
    (
        r#"euler="158.52686 -82.398518 -87.920815""#,
        r#"<compiler eulerseq="XYZ"/>"#,
        [
            0x3fe19a9dbb37086b,
            0x3fdc9acee7dce61f,
            0xbfe33f623c8b16ec,
            0x3fd795f981b12772,
        ],
        true,
    ),
    (
        r#"euler="1.49798 1.01408 -1.260877""#,
        r#"<compiler angle="radian" eulerseq="zyx"/>"#,
        [
            0x3fd49fbba588dc36,
            0xbfe49fb859c3cdfe,
            0xbfb04934a1b34744,
            0x3fe6183a9e7bd616,
        ],
        true,
    ),
    (
        r#"euler="66.323926 -37.197488 99.905972""#,
        r#"<compiler eulerseq="ZYX"/>"#,
        [
            0x3fe49c05304086f3,
            0x3fdfaffc836f032b,
            0xbfe2327a5792cf87,
            0x3fc08928a187f8b1,
        ],
        true,
    ),
    (
        r#"euler="-2.442243 -1.771097 2.569364""#,
        r#"<compiler angle="radian" eulerseq="xYz"/>"#,
        [
            0x3fe849f384139286,
            0xbfdb06901c30ae7d,
            0x3fdfb6d2fc28fd9a,
            0x3f662cf86cf12240,
        ],
        true,
    ),
    (
        r#"euler="-51.108235 -86.25516 109.541262""#,
        r#"<compiler eulerseq="XyZ"/>"#,
        [
            0x3fc1c8cb4eddc1fa,
            0x3fd49eb6626216ed,
            0xbfe39d2e77cfdb5c,
            0x3fe6a78f82ad1b20,
        ],
        true,
    ),
    (
        r#"euler="0.842649 -2.241897 0.328529""#,
        r#"<compiler angle="radian" eulerseq="zxz"/>"#,
        [
            0x3fd7313d1ad37370,
            0xbfebde9bbd71e9b1,
            0xbfcd4e1142386f55,
            0x3fcec2fc8b042fc8,
        ],
        true,
    ),
    (
        r#"euler="59.039659 -120.618631 54.611385""#,
        r#"<compiler eulerseq="YXY"/>"#,
        [
            0x3fd1589aa53facf9,
            0xbfebc72d155f169c,
            0x3fda888e5d8cbe3b,
            0xbfa12f0e7341abf0,
        ],
        true,
    ),
    (
        r#"euler="-2.412631 -1.042776 -2.667469""#,
        r#"<compiler angle="radian" eulerseq="xyz"/>"#,
        [
            0x3fe0cc50a31b1f2d,
            0xbf921b3d975c68b0,
            0xbfea88d7f131559f,
            0xbfc8781eb8a53370,
        ],
        true,
    ),
    (
        r#"euler="-106.408189 172.079832 -34.676712""#,
        r#"<compiler eulerseq="XYZ"/>"#,
        [
            0x3fd1c382be5c6b83,
            0x3fc0095173ba243a,
            0x3fe2c79b6c49d66a,
            0x3fe801ee36d4792c,
        ],
        true,
    ),
    (
        r#"euler="3.138769 -0.392652 0.6882""#,
        r#"<compiler angle="radian" eulerseq="zyx"/>"#,
        [
            0xbfb083382bd25763,
            0x3fc7909048b38254,
            0x3fd528b4920cddea,
            0x3fed8c63c3fa600f,
        ],
        true,
    ),
    (
        r#"euler="134.134338 67.27806 -139.848676""#,
        r#"<compiler eulerseq="ZYX"/>"#,
        [
            0x3fe2e5a270fa1007,
            0xbfc095b3513b497e,
            0x3fe96a7a47ce74b8,
            0x3faef42423f7b368,
        ],
        true,
    ),
    (
        r#"euler="0.570139 0.855868 -2.040333""#,
        r#"<compiler angle="radian" eulerseq="xYz"/>"#,
        [
            0x3fd6df3819f9d49a,
            0xbfca4e0c1fea0ee3,
            0x3fdb4a6c73a28f5c,
            0xbfe9c38bbdc8d9cf,
        ],
        true,
    ),
    (
        r#"euler="-145.9179 134.925811 4.961324""#,
        r#"<compiler eulerseq="XyZ"/>"#,
        [
            0x3fc3419ada0505d6,
            0xbfd82e4b69314ef2,
            0x3fd04abe20265229,
            0xbfec139518b7d6fb,
        ],
        true,
    ),
    (
        r#"euler="-1.959432 -0.303354 -1.803597""#,
        r#"<compiler angle="radian" eulerseq="zxz"/>"#,
        [
            0xbfd357c61d8a1d0b,
            0xbfc3481847eebec3,
            0x3f881636e45b5550,
            0xbfee1e2c00e7553c,
        ],
        true,
    ),
    (
        r#"euler="107.367084 0.172845 -143.12591""#,
        r#"<compiler eulerseq="YXY"/>"#,
        [
            0x3fee745b4f7239d8,
            0xbf4c85fe4f5efa6e,
            0xbfd3a61f6f29dfe5,
            0x3f542eb605e8c2a4,
        ],
        true,
    ),
    (
        r#"euler="2.036019 -2.626974 -1.40372""#,
        r#"<compiler angle="radian" eulerseq="xyz"/>"#,
        [
            0xbfdb7a8d29f1252a,
            0x3fdf910c1048bf8b,
            0xbfcfbc1414e3cbef,
            0xbfe6df7d76ebd53a,
        ],
        true,
    ),
    (
        r#"euler="-169.920934 86.522847 -139.005137""#,
        r#"<compiler eulerseq="XYZ"/>"#,
        [
            0x3fe52def5aa9edcc,
            0xbfc94bd61f13df04,
            0x3fe66b159cf84b66,
            0x3fc6edbdc963407e,
        ],
        true,
    ),
    (
        r#"euler="0.164893 -2.604904 -0.1252""#,
        r#"<compiler angle="radian" eulerseq="zyx"/>"#,
        [
            0x3fd1322194723390,
            0x3fb00e6a998a7901,
            0xbfeebbc21e980f4d,
            0xbfa39f1e4c4f4dc0,
        ],
        true,
    ),
    (
        r#"euler="66.733967 12.54617 -16.139916""#,
        r#"<compiler eulerseq="ZYX"/>"#,
        [
            0x3fea927934037b47,
            0xbfad34207bdb5d6a,
            0x3fc5638bd1d0d178,
            0x3fe0e952b794c9b8,
        ],
        true,
    ),
    (
        r#"euler="-0.096826 -0.402927 0.610858""#,
        r#"<compiler angle="radian" eulerseq="xYz"/>"#,
        [
            0x3fedf5ba1e34a301,
            0xbfbaf6606c675070,
            0xbfc692fd515c263c,
            0x3fd23e174e9d9157,
        ],
        true,
    ),
    (
        r#"euler="-145.852477 -9.173462 -94.945201""#,
        r#"<compiler eulerseq="XyZ"/>"#,
        [
            0x3fd04421cd59e6d0,
            0xbfe52a126e73818b,
            0x3fe5f690c8722147,
            0xbfc4fe2fb6c9746e,
        ],
        true,
    ),
    (
        r#"euler="2.3389 2.809729 -2.774728""#,
        r#"<compiler angle="radian" eulerseq="zxz"/>"#,
        [
            0x3fc4a456b1b5e54f,
            0xbfea50f2f31907a8,
            0x3fe16bfe68eaf2a7,
            0xbfa2487277196ec4,
        ],
        true,
    ),
    (
        r#"euler="-128.413097 73.249514 25.720986""#,
        r#"<compiler eulerseq="YXY"/>"#,
        [
            0x3fe00a9360b0fb0a,
            0x3fc11728226e3b78,
            0xbfe40e4bf51920e7,
            0xbfe29b24600cb01b,
        ],
        true,
    ),
    (
        r#"euler="-3.077722 1.311319 2.841167""#,
        r#"<compiler angle="radian" eulerseq="xyz"/>"#,
        [
            0x3fe366c8715e1ec9,
            0xbfb96c610833154a,
            0x3fe928d3de15e964,
            0xbfb0f059374df0a6,
        ],
        true,
    ),
    (
        r#"euler="-27.680274 -54.903414 24.984555""#,
        r#"<compiler eulerseq="XYZ"/>"#,
        [
            0x3febaed29bcb4d47,
            0xbfbc451b0bbc164a,
            0xbfdee86fa6ce7aee,
            0x3fb426755efac76c,
        ],
        true,
    ),
    (
        r#"euler="2.81047 -1.513353 -1.265375""#,
        r#"<compiler angle="radian" eulerseq="zyx"/>"#,
        [
            0x3fdfcf593e3ad827,
            0x3fde69a9322c4cec,
            0xbfe07d7f4843fb58,
            0x3fe05de36e1a1529,
        ],
        true,
    ),
    (
        r#"euler="-112.064189 46.230315 -11.06727""#,
        r#"<compiler eulerseq="ZYX"/>"#,
        [
            0x3fdeb9a6d07b6fba,
            0xbfd7e96d7db91715,
            0x3fc287d152fda425,
            0xbfe8f8c0b5a69c12,
        ],
        true,
    ),
    (
        r#"euler="-1.561129 -0.717476 -2.977416""#,
        r#"<compiler angle="radian" eulerseq="xYz"/>"#,
        [
            0xbfc888c9d80b6062,
            0x3fc8e85d4ee075ff,
            0xbfe5aaf8d4282a04,
            0xbfe5dd9e0c65b0b1,
        ],
        true,
    ),
    (
        r#"euler="-67.518411 173.846079 30.975776""#,
        r#"<compiler eulerseq="XyZ"/>"#,
        [
            0x3fc878b763239704,
            0xbfd00717557f0a0e,
            0x3fe958a33d640f06,
            0xbfe0bb11efdee1dc,
        ],
        true,
    ),
    (
        r#"euler="-1.391463 2.791233 -0.907035""#,
        r#"<compiler angle="radian" eulerseq="zxz"/>"#,
        [
            0x3fb2418dc4ee1de4,
            0x3fee972729dacd6e,
            0xbfce3b33a11dbbd6,
            0xbfc45b05718a7fa8,
        ],
        true,
    ),
    (
        r#"euler="-89.447513 -167.349697 -127.367592""#,
        r#"<compiler eulerseq="YXY"/>"#,
        [
            0xbfa1cfde312a6279,
            0xbfee14627a4c53e8,
            0xbfbac2b81e22fba2,
            0xbfd4aae5fdc83a1b,
        ],
        true,
    ),
    (
        r#"euler="-1.92615 -1.727774 2.387078""#,
        r#"<compiler angle="radian" eulerseq="xyz"/>"#,
        [
            0xbfdc65e5f70e0558,
            0xbfe333a638cebe92,
            0x3fd57d2ced87c6fc,
            0x3fe2642629253e94,
        ],
        true,
    ),
    (
        r#"euler="-84.7633 -90.503775 37.413543""#,
        r#"<compiler eulerseq="XYZ"/>"#,
        [
            0x3fe4acbde0d332d9,
            0xbfd1ff6148c2ceea,
            0xbfe4c55d75cbea68,
            0xbfd2588c37d63ea6,
        ],
        true,
    ),
    (
        r#"euler="2.66969 -1.335201 1.674164""#,
        r#"<compiler angle="radian" eulerseq="zyx"/>"#,
        [
            0xbfd4be93c5f1674a,
            0x3fe14300460014b5,
            0x3fde1741129982a5,
            0x3fe3cd17d3ec1c5b,
        ],
        true,
    ),
    (
        r#"euler="-47.255091 22.000817 34.501707""#,
        r#"<compiler eulerseq="ZYX"/>"#,
        [
            0x3fec35bcf32408a4,
            0x3fc8ca115578c3cb,
            0x3fd226f759ae9e9c,
            0xbfd4ba81b6beb727,
        ],
        true,
    ),
    (
        r#"euler="2.830792 1.646453 0.052166""#,
        r#"<compiler angle="radian" eulerseq="xYz"/>"#,
        [
            0x3fbfc407177291e4,
            0x3fe594aab2aa143d,
            0x3fb8900b9d8a7e28,
            0xbfe716a78d96de80,
        ],
        true,
    ),
    (
        r#"euler="122.902996 -48.070828 -41.720331""#,
        r#"<compiler eulerseq="XyZ"/>"#,
        [
            0x3fd1f2facfa96b80,
            0x3fe5c56f0279ddf4,
            0xbfddec8d64e39ec2,
            0xbfdf580542219cda,
        ],
        true,
    ),
    (
        r#"euler="2.943846 0.294029 1.75002""#,
        r#"<compiler angle="radian" eulerseq="zxz"/>"#,
        [
            0xbfe62cd17300a09e,
            0x3fbf03fa65b6c32f,
            0x3fb5141fcced4cf3,
            0x3fe696f4adfa2ecf,
        ],
        true,
    ),
    (
        r#"euler="-66.798523 149.504713 40.58421""#,
        r#"<compiler eulerseq="YXY"/>"#,
        [
            0x3fd06498f8156ded,
            0x3fe24802bf6e184a,
            0xbfae890d225a5518,
            0xbfe8e10d07c763c4,
        ],
        true,
    ),
    (
        r#"axisangle="-1.010761 1.734549 -0.124306 -2.247933""#,
        r#"<compiler angle="radian"/>"#,
        [
            0x3fdba7ab95839799,
            0x3fdd00e16f3f90a4,
            0xbfe8e2da5564c676,
            0x3fac890c4ca75e0e,
        ],
        true,
    ),
    (
        r#"axisangle="-0.94899 -1.587016 -1.197429 138.414763""#,
        "",
        [
            0x3fd6b819664c628e,
            0xbfd9c6379da5c7d8,
            0xbfe58d2b41140d28,
            0xbfe042cad974c8c7,
        ],
        true,
    ),
    (
        r#"axisangle="-1.437438 -0.965138 -0.210485 0.625666""#,
        r#"<compiler angle="radian"/>"#,
        [
            0x3fee7267d0871abf,
            0xbfd03b98da0fa0cd,
            0xbfc5cc611bcdb853,
            0xbfa3040e0094628e,
        ],
        true,
    ),
    (
        r#"axisangle="-0.477976 1.140379 -0.972875 -94.607984""#,
        "",
        [
            0x3fe5b3108590de4a,
            0x3fcc946527707eef,
            0xbfe10bf486b5ab2c,
            0x3fdd15ea6521f7c1,
        ],
        true,
    ),
    (
        r#"axisangle="-0.299975 -1.484521 -1.110745 -4.949203""#,
        r#"<compiler angle="radian"/>"#,
        [
            0xbfe92458277faaff,
            0x3fb94b3eb8861724,
            0x3fdf4b32d6bf065a,
            0x3fd76a1f060a4418,
        ],
        true,
    ),
    (
        r#"axisangle="0.901589 -1.413761 0.672296 -200.13331""#,
        "",
        [
            0xbfc65fa4202c1bd3,
            0xbfdf72e6af99103e,
            0x3fe8a8376fcfcc1e,
            0xbfd77363566c1670,
        ],
        true,
    ),
    (
        r#"axisangle="0.587806 0.626121 -0.586095 6.06333""#,
        r#"<compiler angle="radian"/>"#,
        [
            0xbfefce8dab44036b,
            0x3fafc14b431cc85c,
            0x3fb0e997d9e4ef5d,
            0xbfafa9a18c3d8cb0,
        ],
        true,
    ),
    (
        r#"axisangle="0.627159 0.22917 -1.814842 -67.297866""#,
        "",
        [
            0x3feaa36aa75b2c02,
            0xbfc700967e6498a7,
            0xbfb0cf7e9d57f9ea,
            0x3fe0a406dc117ba4,
        ],
        true,
    ),
    (
        r#"axisangle="-0.935739 -0.120566 1.73332 5.095412""#,
        r#"<compiler angle="radian"/>"#,
        [
            0xbfea854ce95ce51d,
            0xbfd0fb3d7664ba16,
            0xbfa180fcce8f7125,
            0x3fdf74a0cccb314e,
        ],
        true,
    ),
    (
        r#"axisangle="-1.752379 0.720503 1.736808 30.557886""#,
        "",
        [
            0x3feede72c219a811,
            0xbfc6ff255433c716,
            0x3fb2e915491d163a,
            0x3fc6cad5b4a66a87,
        ],
        true,
    ),
    (
        r#"axisangle="-1.61299 1.467125 -0.29464 6.727692""#,
        r#"<compiler angle="radian"/>"#,
        [
            0xbfef3680d316edad,
            0x3fc4af2e4a709e96,
            0xbfc2d0547c749b43,
            0x3f9e3a0985e776bd,
        ],
        true,
    ),
    (
        r#"axisangle="1.497868 1.315507 1.724923 -310.949982""#,
        "",
        [
            0xbfed1ceb5045143c,
            0xbfce3061a590c7e9,
            0xbfca83794de789c3,
            0xbfd161f2212559ca,
        ],
        true,
    ),
    (
        r#"axisangle="1.175105 -0.779269 -1.459271 -1.692013""#,
        r#"<compiler angle="radian"/>"#,
        [
            0x3fe5371de06db0e6,
            0xbfdbbf1444bac84f,
            0x3fd2666545194ec0,
            0x3fe13a61176a14be,
        ],
        true,
    ),
    (
        r#"axisangle="0.355625 -1.980508 -0.201395 204.668394""#,
        "",
        [
            0xbfcb57afe658d47c,
            0x3fc5fd7b80b285bf,
            0xbfee9dc668fa5db0,
            0xbfb8e818a3db2240,
        ],
        true,
    ),
    (
        r#"axisangle="-0.018833 0.112595 -0.814819 6.60001""#,
        r#"<compiler angle="radian"/>"#,
        [
            0xbfef996d8533556c,
            0x3f6d947b8058f719,
            0xbf961b1b53c0cea5,
            0x3fc3ff2cb1ae3adf,
        ],
        true,
    ),
    (
        r#"axisangle="-1.007309 0.724609 0.583352 -359.585129""#,
        "",
        [
            0xbfeffff2417a4c66,
            0x3f65c9e375789560,
            0xbf5f58ec1a3fdcec,
            0xbf593c865d50b37b,
        ],
        true,
    ),
    (
        r#"axisangle="1.985984 -1.058154 1.955354 5.996228""#,
        r#"<compiler angle="radian"/>"#,
        [
            0xbfefabd2e3e4852e,
            0x3fb862a50f558d4f,
            0xbfa9fc4c80b95c22,
            0x3fb8025d2a28473d,
        ],
        true,
    ),
    (
        r#"axisangle="1.603875 1.421024 -0.408994 -39.301785""#,
        "",
        [
            0x3fee22e4052bb5cd,
            0xbfcfa5a9feff709b,
            0xbfcc0a07122f8ae7,
            0x3fb023e873cc1a0b,
        ],
        true,
    ),
    (
        r#"axisangle="-0.617009 1.743994 -1.882966 -5.083994""#,
        r#"<compiler angle="radian"/>"#,
        [
            0xbfea6b04e5ad107f,
            0x3fc0e24313130618,
            0xbfd7dc80031b9fcc,
            0x3fd9c34326de98ec,
        ],
        true,
    ),
    (
        r#"axisangle="1.82564 1.225838 0.824164 336.883066""#,
        "",
        [
            0xbfef59df5527c988,
            0x3fc3f02a30465340,
            0x3fbac6733bd10924,
            0x3fb2006e96100f02,
        ],
        true,
    ),
    (
        r#"axisangle="0.094506 1.341063 -0.22854 5.60974""#,
        r#"<compiler angle="radian"/>"#,
        [
            0xbfee33f54c57383d,
            0x3f97725d519de9a2,
            0x3fd4cb701299c460,
            0xbfac59a2f13d66a2,
        ],
        true,
    ),
    (
        r#"axisangle="-1.721396 -0.246719 0.30806 211.921675""#,
        "",
        [
            0xbfd19946c8aeccd0,
            0xbfedfcff618ab7fa,
            0xbfc13138a44221fe,
            0x3fc5777bcdb91af5,
        ],
        true,
    ),
    (
        r#"axisangle="-0.309975 0.72238 1.8712 -1.13617""#,
        r#"<compiler angle="radian"/>"#,
        [
            0x3feaf94e4325f71f,
            0x3fb50913f6f8a072,
            0xbfc882d115b58972,
            0xbfdfbee96b232ef7,
        ],
        true,
    ),
    (
        r#"axisangle="-1.201814 -1.878871 1.447229 -17.254671""#,
        "",
        [
            0x3fefa34e91eb8b7b,
            0x3fb15bc1ec19dfbb,
            0x3fbb23343e0dfefe,
            0xbfb4e730923beecb,
        ],
        true,
    ),
    (
        r#"axisangle="-0.448023 0.078678 -1.518063 -3.89123""#,
        r#"<compiler angle="radian"/>"#,
        [
            0xbfd76e3e9edf5549,
            0x3fd0d654be98da52,
            0xbfa7a78cdebd6385,
            0x3fec8679c17fccce,
        ],
        true,
    ),
    (
        r#"axisangle="-1.05687 -0.979578 1.438354 -182.434481""#,
        "",
        [
            0xbf95c0cace769938,
            0x3fe09b60eaacc3e7,
            0x3fdec8ecf51e645a,
            0xbfe699f0e093d443,
        ],
        true,
    ),
    (
        r#"axisangle="-1.638105 1.466859 0.233038 -6.607996""#,
        r#"<compiler angle="radian"/>"#,
        [
            0xbfef943405afb8a6,
            0xbfbeaa45c2b683ba,
            0x3fbb759cb852e7a3,
            0x3f91732448d0f94a,
        ],
        true,
    ),
    (
        r#"axisangle="0.700731 0.744952 0.598411 197.704157""#,
        "",
        [
            0xbfc3b278acee4845,
            0x3fe2b2c4c591ed29,
            0x3fe3e0d8d748c460,
            0x3fdfef9e968683d6,
        ],
        true,
    ),
    (
        r#"axisangle="-0.30428 -1.100463 0.97706 1.750917""#,
        r#"<compiler angle="radian"/>"#,
        [
            0x3fe48029acc12ca2,
            0xbfc3e68e90abb207,
            0xbfe1fe4299c27ed5,
            0x3fdff373e6cf1396,
        ],
        true,
    ),
    (
        r#"axisangle="0.815032 -1.265404 -0.509222 137.617301""#,
        "",
        [
            0x3fd7228cf70a0873,
            0x3fde9b91f0cd9e0e,
            0xbfe7c2aeb6890c91,
            0xbfd31f916fa34594,
        ],
        true,
    ),
    (
        r#"zaxis="-0.46172 0.409849 -0.108117""#,
        "",
        [
            0x3fe4956267a4d75d,
            0xbfe043e5cafd8bfe,
            0xbfe252e2bd35795c,
            0x0000000000000000,
        ],
        true,
    ),
    (
        r#"zaxis="1.018259 -0.606488 -0.008424""#,
        "",
        [
            0x3fe68bff1616d7f9,
            0x3fd73d7224bd7f5e,
            0x3fe382642d85ce0a,
            0x8000000000000000,
        ],
        true,
    ),
    (
        r#"zaxis="1.313979 -0.757513 -1.297063""#,
        "",
        [
            0x3fdac68d5f06c293,
            0x3fdd086a3beb4200,
            0x3fe92e1e5923b7a6,
            0x8000000000000000,
        ],
        true,
    ),
    (
        r#"zaxis="0.546444 -1.467526 1.428055""#,
        "",
        [
            0x3fed464697fd3c18,
            0x3fd838a48a9d8210,
            0x3fc209b582240fce,
            0x8000000000000000,
        ],
        true,
    ),
    (
        r#"zaxis="-0.90014 1.616564 0.135103""#,
        "",
        [
            0x3fe76fd52babd0f6,
            0xbfe3092c5791c7de,
            0xbfd532fd60adf589,
            0x0000000000000000,
        ],
        true,
    ),
    (
        r#"zaxis="-1.383377 0.084101 0.73007""#,
        "",
        [
            0x3feb65c3d5e4e194,
            0xbfa00d99bffa77d3,
            0xbfe080ecd9004069,
            0x0000000000000000,
        ],
        true,
    ),
    (
        r#"zaxis="-0.055445 0.540579 -0.523998""#,
        "",
        [
            0x3fd907489813cfaf,
            0xbfed4c40473e2969,
            0xbfb80a29c57f85da,
            0x0000000000000000,
        ],
        true,
    ),
    (
        r#"zaxis="1.836773 -1.302191 1.190943""#,
        "",
        [
            0x3feb695c28f031ba,
            0x3fd3192482b4a95a,
            0x3fdaf0420f10fb4e,
            0x8000000000000000,
        ],
        true,
    ),
    (
        r#"zaxis="-1.90506 0.028644 -1.73406""#,
        "",
        [
            0x3fd9dfeba1779198,
            0xbf8c2944ff0269f5,
            0xbfed43cd44c2a334,
            0x0000000000000000,
        ],
        true,
    ),
    (
        r#"zaxis="-1.473 0.353976 1.617329""#,
        "",
        [
            0x3fedc2a193206059,
            0xbfb5fc18741f9b02,
            0xbfd6df0c1320cf80,
            0x0000000000000000,
        ],
        true,
    ),
    (
        r#"zaxis="-0.762911 -0.388913 -0.095897""#,
        "",
        [
            0x3fe554c77e9c9b37,
            0x3fd5aab2c0ff6f6d,
            0xbfe54056c7f76cfb,
            0x0000000000000000,
        ],
        true,
    ),
    (
        r#"zaxis="0.420239 -0.598763 -0.594314""#,
        "",
        [
            0x3fdb81a5abfa8408,
            0x3fe7a671301ad355,
            0x3fe099482330203d,
            0x8000000000000000,
        ],
        true,
    ),
    (
        r#"zaxis="1.413857 0.222511 -0.590134""#,
        "",
        [
            0x3fe1ccbe60d9f6a5,
            0xbfc08973262be0a8,
            0x3fea44f0ae045c4c,
            0x0000000000000000,
        ],
        true,
    ),
    (
        r#"zaxis="1.272311 1.996074 -0.27116""#,
        "",
        [
            0x3fe54d091f93e759,
            0xbfe4232d0c848631,
            0x3fd9abe341183396,
            0x0000000000000000,
        ],
        true,
    ),
    (
        r#"zaxis="-1.939152 -0.778752 1.592408""#,
        "",
        [
            0x3fecad1ee3d8ebca,
            0x3fc52b4ae32b99ee,
            0xbfda5b39ee934b8e,
            0x0000000000000000,
        ],
        true,
    ),
    (
        r#"zaxis="-1.312619 -1.205154 1.169362""#,
        "",
        [
            0x3fec28973434993b,
            0x3fd48ff2acc34033,
            0xbfd66557232608b7,
            0x0000000000000000,
        ],
        true,
    ),
    (
        r#"zaxis="-1.542718 1.397797 -1.094989""#,
        "",
        [
            0x3fe08ae20e4e8150,
            0xbfe26477575189ea,
            0xbfe44ca1974cc84d,
            0x0000000000000000,
        ],
        true,
    ),
    (
        r#"zaxis="-0.448389 0.377521 -1.551522""#,
        "",
        [
            0x3fc6fe0b3336306d,
            0xbfe4465ee3e4ac19,
            0xbfe814b426d0df16,
            0x0000000000000000,
        ],
        true,
    ),
    (
        r#"zaxis="1.099949 -0.24618 1.081094""#,
        "",
        [
            0x3fed6f50b6661d7c,
            0x3fb5ef2a117551c1,
            0x3fd8803befabd91a,
            0x8000000000000000,
        ],
        true,
    ),
    (
        r#"zaxis="1.056194 -1.692857 1.290107""#,
        "",
        [
            0x3fec1b5abe161df6,
            0x3fd9f4ead6250606,
            0x3fd031d82e82fd94,
            0x8000000000000000,
        ],
        true,
    ),
    (
        r#"xyaxes="1.283336 -1.033076 0.365215 -1.177895 0.154067 1.007359""#,
        "",
        [
            0x3fe09dbb84cff2b1,
            0x3fe9017284dd86b2,
            0xbfd579f62ea31270,
            0xbfb5a1f1782d5712,
        ],
        false,
    ),
    (
        r#"xyaxes="-0.492608 0.131915 -1.474329 1.452517 0.505538 1.321581""#,
        "",
        [
            0x3fdffa0ed97e3ba2,
            0x3fd3766e7f43d3b1,
            0x3fe74927c4a36bb8,
            0xbfd6ee137a35abd3,
        ],
        false,
    ),
    (
        r#"xyaxes="-1.677172 0.9814 -1.638237 0.225968 -1.076983 1.606179""#,
        "",
        [
            0x3fd65c81323e7dca,
            0xbfcc09c165513c17,
            0x3fda762b65b64503,
            0x3fe9fa1d496d6f38,
        ],
        false,
    ),
    (
        r#"xyaxes="-1.575074 -0.108034 -0.116538 -0.456372 -1.741758 0.664381""#,
        "",
        [
            0xbf9b06e7cd300e54,
            0xbfa5e54c246d5289,
            0x3fc8b8566ceee76d,
            0x3fef5b3a68302819,
        ],
        false,
    ),
    (
        r#"xyaxes="1.888048 -0.556149 1.595198 -0.644899 -0.907271 -0.18103""#,
        "",
        [
            0x3fb6174a706c7929,
            0x3fedc3bdd6b6f3bc,
            0xbfc2f5d47710730f,
            0x3fd4c839bb26d3f2,
        ],
        false,
    ),
    (
        r#"xyaxes="1.891225 1.117723 0.530269 -1.841435 -0.19456 -1.940517""#,
        "",
        [
            0x3feab1a055b89881,
            0xbfde317b0beafb68,
            0xbfcde6278edd4ada,
            0x3fc50943e6f69947,
        ],
        false,
    ),
    (
        r#"xyaxes="-1.751914 -0.494536 -1.023956 -1.583899 0.046702 1.129531""#,
        "",
        [
            0x3fbe8905796ec5b9,
            0xbfd07d072d6830ee,
            0x3fe7b7b1b7f5fd6e,
            0x3fe3776832160781,
        ],
        false,
    ),
    (
        r#"xyaxes="1.702835 1.664181 -0.295662 -1.286439 1.854591 -0.30677""#,
        "",
        [
            0x3fed7c33761ad8b2,
            0xbfb39ac228b37d55,
            0x3fa21889631603e4,
            0x3fd846dd3a760282,
        ],
        false,
    ),
    (
        r#"xyaxes="-0.62798 1.29664 0.232319 1.809631 0.804747 1.783736""#,
        "",
        [
            0x3fc01893ea386d09,
            0x3fe098fe17116ce5,
            0x3fe8c34bba818b87,
            0x3fd5d438ef0612c4,
        ],
        false,
    ),
    (
        r#"xyaxes="-1.387362 1.773059 0.166433 0.786394 -0.796226 -1.023494""#,
        "",
        [
            0xbfd403e75cbb634e,
            0x3fd3b796ba322e14,
            0x3fe600d6fad3d980,
            0xbfe281cf63534850,
        ],
        false,
    ),
    (
        r#"xyaxes="-0.813255 -1.320065 1.8426 -1.522509 -1.029196 0.534019""#,
        "",
        [
            0xbfc223a89e4dee4a,
            0xbfe1d8ead55f7021,
            0x3fe402dd05048301,
            0xbfe0ddaf726cd4d7,
        ],
        false,
    ),
    (
        r#"xyaxes="-1.1816 -1.145293 -0.988098 -1.063579 -1.959079 0.960018""#,
        "",
        [
            0xbfc360ace60ede6f,
            0xbfda550443067e2f,
            0x3fdbe60d9b092e59,
            0x3fe926cfa2bb035b,
        ],
        false,
    ),
    (
        r#"xyaxes="0.636803 -0.759367 1.596852 0.237409 -1.119957 1.860167""#,
        "",
        [
            0x3fd2b85555b3fa6e,
            0x3fe87392afc54725,
            0xbfdaa38c4de73323,
            0x3fd962bfdf4eb0df,
        ],
        false,
    ),
    (
        r#"xyaxes="-1.081214 1.238279 1.266159 -1.988361 1.388249 -0.317294""#,
        "",
        [
            0x3fdf4f1e8c44e63c,
            0x3f88757f7c73a38c,
            0xbfe378291e714831,
            0x3fe3fe4a71072162,
        ],
        false,
    ),
    (
        r#"xyaxes="0.008844 1.783406 -0.193531 -1.033393 -1.802004 1.541916""#,
        "",
        [
            0x3fe4d95efbb58dc3,
            0x3fd1dedb330cc2ce,
            0x3fd627c2a4927112,
            0x3fe3aa8f44b8a5ac,
        ],
        false,
    ),
    (
        r#"xyaxes="-0.180373 0.844668 0.563609 1.919786 1.744809 1.743235""#,
        "",
        [
            0xbfbf4eae7c8f846f,
            0x3fe42d7eaa8fd7f2,
            0x3fe69ed1a06b3bb3,
            0x3fd2f590607dbe48,
        ],
        false,
    ),
    (
        r#"xyaxes="0.01798 1.641191 -0.773665 -1.503152 0.557617 -1.885916""#,
        "",
        [
            0x3fe2b43e53d21f88,
            0xbfd9dd12976af110,
            0xbfbd7b11a7b9dada,
            0x3fe6361ed2669308,
        ],
        false,
    ),
    (
        r#"xyaxes="1.889289 -0.700396 -0.303389 -0.836705 -0.708252 0.149961""#,
        "",
        [
            0x3f964c9b8e2983df,
            0x3fef67867418e86e,
            0xbfc6307c5e532435,
            0xbfb46782a1342705,
        ],
        false,
    ),
    (
        r#"xyaxes="1.204239 -1.94244 -0.627465 -1.013668 1.83416 -0.244301""#,
        "",
        [
            0x3fe636bf42400109,
            0xbfe0b1bc8d85f6d0,
            0x3fd9f1c81d46e975,
            0xbfd24870d430d422,
        ],
        false,
    ),
    (
        r#"xyaxes="-0.222904 1.749027 -0.451955 -0.461949 -0.402946 -1.053193""#,
        "",
        [
            0x3fe00cf53b115799,
            0xbfdbb0815c4df88f,
            0xbfd53cad6b3a5bf2,
            0x3fe57e33ef984315,
        ],
        false,
    ),
    (
        r#"quat="0.815761 -0.852329 0.037679 0.286408""#,
        "",
        [
            0x3fe57dc0480278ac,
            0xbfe67460968b338a,
            0x3f9fc3cff41c8a80,
            0x3fce2e817d771791,
        ],
        false,
    ),
    (
        r#"quat="1.066728 0.96007 -0.075114 1.799375""#,
        "",
        [
            0x3fdda577f2ba21b3,
            0x3fdaaea08273d04b,
            0xbfa0b351b89447b3,
            0x3fe90105c5e74fb7,
        ],
        false,
    ),
    (
        r#"quat="1.78284 1.751239 0.856732 -0.182698""#,
        "",
        [
            0x3fe58b2b8ef89ce8,
            0x3fe52969cb7efad9,
            0x3fd4b48ea8a10533,
            0xbfb1a95f75ee65a2,
        ],
        false,
    ),
    (
        r#"quat="-0.560411 1.441117 -0.105493 0.992469""#,
        "",
        [
            0xbfd37d0fba5febad,
            0x3fe90ec67855a1f8,
            0xbfad593719f8ee1c,
            0x3fe141bad808778a,
        ],
        false,
    ),
    (
        r#"quat="-0.449678 -0.538462 -1.928065 0.920664""#,
        "",
        [
            0xbfc99855f7b5e80e,
            0xbfcea605d6ec09ca,
            0xbfeb6f8ac2f93450,
            0x3fda3393bbdc6ce0,
        ],
        false,
    ),
    (
        r#"quat="-0.840714 -0.023822 0.902955 1.32774""#,
        "",
        [
            0xbfddaf172b5ac9fb,
            0xbf8aea5aa0f987bb,
            0x3fdfe1ad8287184c,
            0x3fe7709f9fec4405,
        ],
        false,
    ),
    (
        r#"quat="1.778859 0.654026 -1.068487 1.715424""#,
        "",
        [
            0x3fe48b9688fb5480,
            0x3fce37164b39b0a1,
            0xbfd8ae6f0eccb492,
            0x3fe3d0074b4eef43,
        ],
        false,
    ),
    (
        r#"quat="-1.782456 -0.349909 1.700102 1.381545""#,
        "",
        [
            0xbfe40b056f21dd4f,
            0xbfbf7a0d7190f0a4,
            0x3fe31df47ac10b08,
            0x3fdf11e5ebbbb65b,
        ],
        false,
    ),
    (
        r#"quat="0.378541 1.473572 -1.291706 -1.842717""#,
        "",
        [
            0x3fc1d65a4ec5059b,
            0x3fe15befe5d0e0b3,
            0xbfde6ef245abcd76,
            0xbfe5b530711b67a8,
        ],
        false,
    ),
    (
        r#"quat="-0.812983 1.721233 -1.8409 -0.698379""#,
        "",
        [
            0xbfd2ffab84fc4499,
            0x3fe41ca32a57c1b6,
            0xbfe582975a2d874a,
            0xbfd0520d3af18d47,
        ],
        false,
    ),
];

#[test]
fn seeded_orientations_are_mujocos() {
    let here = cfg!(all(target_os = "macos", target_arch = "aarch64"));
    let mut differ = Vec::new();
    for &(attr, compiler, bits, libm) in SEEDED {
        let mujoco = bits.map(f64::from_bits);
        let got = quat_of(Element::Body, attr, compiler);
        if !is_mujocos(got, mujoco, (libm && !here).then_some(1e-14)) {
            differ.push(format!("{attr} {compiler}: {got:?}, MuJoCo {mujoco:?}"));
        }
    }
    assert!(
        differ.is_empty(),
        "{} of {} differ:\n  {}",
        differ.len(),
        SEEDED.len(),
        differ.join("\n  ")
    );
}

/// A class's alternative orientation and the element's or child class's own,
/// as MuJoCo 3.5.0's reader layers them: it keeps one alternative per element,
/// the last it reads (`ReadAlternative`, `xml_base.cc:54-77`), and an element
/// starts from its class's values, so a layer that writes any alternative
/// takes none it inherits, while a written `quat` does not displace an
/// inherited alternative (the last two). `(element, defaults, attributes,
/// what, MuJoCo's quaternion as bits)`.
const LAYERED: &[(&str, &str, &str, &str, [u64; 4])] = &[
    (
        "geom",
        r#"<default class="c"><geom euler="0 90 0"/></default>"#,
        r#"class="c" axisangle="1 0 0 90""#,
        "an element's axisangle over its class's euler",
        [
            0x3fe6a09e667f3bcd,
            0x3fe6a09e667f3bcc,
            0x0000000000000000,
            0x0000000000000000,
        ],
    ),
    (
        "geom",
        r#"<default class="p"><geom euler="0 90 0"/><default class="c"><geom axisangle="1 0 0 90"/></default></default>"#,
        r#"class="c""#,
        "a child class's axisangle over its parent's euler",
        [
            0x3fe6a09e667f3bcd,
            0x3fe6a09e667f3bcc,
            0x0000000000000000,
            0x0000000000000000,
        ],
    ),
    (
        "site",
        r#"<default class="c"><site euler="0 90 0"/></default>"#,
        r#"class="c" zaxis="0 1 0""#,
        "an element's zaxis over its class's euler",
        [
            0x3fe6a09e667f3bcd,
            0xbfe6a09e667f3bcc,
            0x0000000000000000,
            0x0000000000000000,
        ],
    ),
    (
        "site",
        r#"<default class="p"><site xyaxes="0 1 0 -1 0 0"/><default class="c"><site zaxis="1 0 0"/></default></default>"#,
        r#"class="c""#,
        "a child class's zaxis over its parent's xyaxes",
        [
            0x3fe6a09e667f3bcd,
            0x0000000000000000,
            0x3fe6a09e667f3bcc,
            0x0000000000000000,
        ],
    ),
    (
        "geom",
        r#"<default class="c"><geom axisangle="0 0 1 30"/></default>"#,
        r#"class="c" xyaxes="0 1 0 -1 0 0""#,
        "an element's xyaxes over its class's axisangle",
        [
            0x3fe6a09e667f3bcd,
            0x0000000000000000,
            0x0000000000000000,
            0x3fe6a09e667f3bcc,
        ],
    ),
    (
        "geom",
        r#"<default class="p"><geom axisangle="0 0 1 30"/><default class="c"><geom zaxis="0 1 0"/></default></default>"#,
        r#"class="c""#,
        "a child class's zaxis over its parent's axisangle",
        [
            0x3fe6a09e667f3bcd,
            0xbfe6a09e667f3bcc,
            0x0000000000000000,
            0x0000000000000000,
        ],
    ),
    (
        "site",
        r#"<default class="c"><site xyaxes="0 1 0 -1 0 0"/></default>"#,
        r#"class="c" axisangle="1 0 0 90""#,
        "an element's axisangle over its class's xyaxes",
        [
            0x3fe6a09e667f3bcd,
            0x3fe6a09e667f3bcc,
            0x0000000000000000,
            0x0000000000000000,
        ],
    ),
    (
        "geom",
        r#"<default class="c"><geom zaxis="1 0 0"/></default>"#,
        r#"class="c" euler="90 0 0""#,
        "an element's euler over its class's zaxis",
        [
            0x3fe6a09e667f3bcd,
            0x3fe6a09e667f3bcc,
            0x0000000000000000,
            0x0000000000000000,
        ],
    ),
    (
        "geom",
        r#"<default class="c"><geom euler="0 90 0"/></default>"#,
        r#"class="c" quat="0 1 0 0""#,
        "a class's euler over an element's quat",
        [
            0x3fe6a09e667f3bcd,
            0x0000000000000000,
            0x3fe6a09e667f3bcc,
            0x0000000000000000,
        ],
    ),
    (
        "geom",
        r#"<default class="p"><geom euler="0 90 0"/><default class="c"><geom quat="0 1 0 0"/></default></default>"#,
        r#"class="c""#,
        "a parent class's euler over a child class's quat",
        [
            0x3fe6a09e667f3bcd,
            0x0000000000000000,
            0x3fe6a09e667f3bcc,
            0x0000000000000000,
        ],
    ),
];

#[test]
fn a_class_alternative_is_layered_as_mujocos_reader_layers_it() {
    let here = cfg!(all(target_os = "macos", target_arch = "aarch64"));
    let mut differ = Vec::new();
    for &(element, defaults, attrs, what, bits) in LAYERED {
        let inner = if element == "geom" {
            format!(r#"<geom type="sphere" size="0.1" {attrs}/>"#)
        } else {
            format!(r#"<geom type="sphere" size="0.1"/><site name="s" {attrs}/>"#)
        };
        let model = load_model(&format!(
            r#"<mujoco><default>{defaults}</default><worldbody><body name="b"><joint type="hinge"/>{inner}</body></worldbody></mujoco>"#
        ))
        .unwrap_or_else(|e| panic!("{what}: {e}"));
        let q = if element == "geom" {
            model.geom_quat[0]
        } else {
            model.site_quat[0]
        }
        .into_inner();
        let got = [q.w, q.i, q.j, q.k];
        let mujoco = bits.map(f64::from_bits);
        if !is_mujocos(got, mujoco, (!here).then_some(1e-14)) {
            differ.push(format!("{what}: {got:?}, MuJoCo {mujoco:?}"));
        }
    }
    assert!(
        differ.is_empty(),
        "{} of {} differ:\n  {}",
        differ.len(),
        LAYERED.len(),
        differ.join("\n  ")
    );
}

/// A joint's written axis made unit once, as MuJoCo 3.5.0 makes it
/// (`mjuu_normvec` in `mjCJoint::Compile`, `user_objects.cc:2957-2960`; ledger
/// L146): its `jnt_axis` as bits, the same for a hinge and a slide. A sum, a
/// square root and divisions only, so bit for bit on every platform. L146's
/// axes, a unit one, two whose norm is exact, 20 drawn with Python's
/// `random.Random(146)`, and three within mjEPS of unit length, which MuJoCo
/// keeps as written.
const JOINT_AXES: &[(&str, [u64; 3])] = &[
    (
        "1 0 1",
        [0x3fe6a09e667f3bcc, 0x0000000000000000, 0x3fe6a09e667f3bcc],
    ),
    (
        "1 1 0",
        [0x3fe6a09e667f3bcc, 0x3fe6a09e667f3bcc, 0x0000000000000000],
    ),
    (
        "1 -1 0",
        [0x3fe6a09e667f3bcc, 0xbfe6a09e667f3bcc, 0x0000000000000000],
    ),
    (
        "0 1 0.3",
        [0x0000000000000000, 0x3feea6834f48c4b7, 0x3fd263e862c542d4],
    ),
    (
        "0.3 -0.7 1.1",
        [0x3fccb395ffe349b9, 0xbfe0be177fef4057, 0x3fea4f497fe5ae40],
    ),
    (
        "0.3 1 0.2",
        [0x3fd20fd4ca53f295, 0x3fee1a62a68be9a4, 0x3fc8151bb86fee1d],
    ),
    (
        "0 0 1",
        [0x0000000000000000, 0x0000000000000000, 0x3ff0000000000000],
    ),
    (
        "3 4 0",
        [0x3fe3333333333333, 0x3fe999999999999a, 0x0000000000000000],
    ),
    (
        "0 0 2",
        [0x0000000000000000, 0x0000000000000000, 0x3ff0000000000000],
    ),
    (
        "-2.199989 0.739929 1.635676",
        [0xbfe8caf3fcbe31ef, 0x3fd0ad614f0beb64, 0x3fe26ee9f9b68876],
    ),
    (
        "-2.608998 0.382392 2.38984",
        [0xbfe775ccd5e3e40d, 0x3fbb81ff8ee420d8, 0x3fe57d4ed4ebb3fd],
    ),
    (
        "-0.917843 -2.762746 2.848001",
        [0xbfccd8e13dce590d, 0xbfe5b5331d86c499, 0x3fe660b029431fc1],
    ),
    (
        "-0.517664 1.767707 1.242195",
        [0xbfcdd32ae61a9252, 0x3fe9761924418bcc, 0x3fe1e45e4528708b],
    ),
    (
        "-0.441513 2.880307 2.632335",
        [0xbfbcc877a02c62c4, 0x3fe778ba0e82817e, 0x3fe5736c15aa4c1b],
    ),
    (
        "-2.555888 -2.770968 2.475812",
        [0xbfe2227e2e174a2e, 0xbfe3a9298f7789e6, 0x3fe1910b252b8dfc],
    ),
    (
        "-0.336757 -2.804286 2.701321",
        [0xbfb60eed68b2ae66, 0xbfe6f5fa2136ca34, 0x3fe61e27c313f717],
    ),
    (
        "1.339977 1.652151 -1.834662",
        [0x3fde8756ab55b779, 0x3fe2d209a6a5972d, 0xbfe4e647b727c401],
    ),
    (
        "0.475137 -1.686601 2.294746",
        [0x3fc5106ba65b39ad, 0xbfe2b163a0c41f5b, 0x3fe96ee0a9fc4f1c],
    ),
    (
        "-0.662856 0.890793 -0.288305",
        [0xbfe27d7810acff69, 0x3fe8d92c0aed0286, 0xbfd015959ef4f9fb],
    ),
    (
        "2.988966 2.308089 -0.037354",
        [0x3fe953888bf603fc, 0x3fe38e9ae0948e01, 0xbf8441b4a803f718],
    ),
    (
        "2.533646 -0.955432 0.956737",
        [0x3fec3b4078cbb9f3, 0xbfd54ac277f46360, 0x3fd5523466e92617],
    ),
    (
        "-2.351602 -1.982212 -1.956664",
        [0xbfe4a4cc30ca60a9, 0xbfe166a92a882bed, 0xbfe12d3f0f121a48],
    ),
    (
        "-0.312274 -0.84184 -2.209465",
        [0xbfc0c2806297402b, 0xbfd6973f6977922f, 0xbfeda54245c54196],
    ),
    (
        "2.224664 -0.487036 -2.854058",
        [0x3fe37f393e6f82cf, 0xbfc112d3cf4a8458, 0xbfe90352a1c3accf],
    ),
    (
        "1.650887 -0.439885 -2.565537",
        [0x3fe12393bdaeeace, 0xbfc244585c762512, 0xbfeaa27359435551],
    ),
    (
        "-2.216432 -0.372246 1.000409",
        [0xbfecd4ae7f47929a, 0xbfc35e4abaa452b7, 0x3fda06b0cb4e72a3],
    ),
    (
        "-0.234738 2.839575 0.572968",
        [0xbfb4ad4172132558, 0x3fef43ebee82106c, 0x3fc93c1be30a9cfb],
    ),
    (
        "1.554873 -0.973123 0.191181",
        [0x3feafab92f932413, 0xbfe0e29929d2b942, 0x3fba89c930bb47b6],
    ),
    (
        "-2.27121 -1.149744 -2.454567",
        [0xbfe48d67f508a540, 0xbfd4cee9f641af4a, 0xbfe6362a1eb1a708],
    ),
    (
        "0.7071067811865476 0 0.7071067811865476",
        [0x3fe6a09e667f3bcd, 0x0000000000000000, 0x3fe6a09e667f3bcd],
    ),
    (
        "0 0.7071067811865489 0.7071067811865489",
        [0x0000000000000000, 0x3fe6a09e667f3bd9, 0x3fe6a09e667f3bd9],
    ),
    (
        "0.6 0.8 1e-9",
        [0x3fe3333333333333, 0x3fe999999999999a, 0x3e112e0be826d695],
    ),
];

#[test]
fn a_joint_axis_is_made_unit_once_as_mujocos_is() {
    let mut differ = Vec::new();
    for &(axis, bits) in JOINT_AXES {
        for kind in ["hinge", "slide"] {
            let model = load_model(&format!(
                r#"<mujoco><worldbody><body name="b"><joint type="{kind}" axis="{axis}"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#
            ))
            .unwrap_or_else(|e| panic!("{kind} {axis}: {e}"));
            let got = model.jnt_axis[0];
            let mujoco = bits.map(f64::from_bits);
            if (0..3).any(|k| got[k].to_bits() != mujoco[k].to_bits()) {
                differ.push(format!("{kind} {axis}: {got:?}, MuJoCo {mujoco:?}"));
            }
        }
    }
    assert!(
        differ.is_empty(),
        "{} of {} differ:\n  {}",
        differ.len(),
        2 * JOINT_AXES.len(),
        differ.join("\n  ")
    );
}

/// A joint axis too small to make unit (`normvec` returns 0 below a norm of
/// 1e-7) falls back to z. MuJoCo 3.5.0 refuses both ("axis too small in
/// joint"); that refusal is the Rigid-loading book's L20 (pull request RL-6).
#[test]
fn a_joint_axis_too_small_to_make_unit_falls_back_to_z() {
    for axis in ["0 0 0", "1e-8 0 0"] {
        let model = load_model(&format!(
            r#"<mujoco><worldbody><body name="b"><joint type="hinge" axis="{axis}"/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>"#
        ))
        .unwrap();
        assert_eq!(
            [
                model.jnt_axis[0].x,
                model.jnt_axis[0].y,
                model.jnt_axis[0].z
            ]
            .map(f64::to_bits),
            [0.0_f64, 0.0, 1.0].map(f64::to_bits),
            "{axis}"
        );
    }
}

/// A frame's turn and its geom's: MuJoCo 3.5.0's `geom_quat`, within 1e-14 (the
/// composition is not MuJoCo's arithmetic: ledger L255).
const FRAME_CASES: &[(&str, &str, [u64; 4])] = &[
    // frame_quat_1234_geom
    (
        "frame_quat_1234_geom",
        r#"<frame quat="1 2 3 4"><geom type="box" size="0.1 0.2 0.3" euler="10 20 30"/></frame>"#,
        [
            0xbfc32c40147f2818,
            0x3fda3019cbcd00f6,
            0x3fe13ba9cb166e63,
            0x3fe713f9909ff979,
        ],
    ),
    // frame_euler_geom
    (
        "frame_euler_geom",
        r#"<frame euler="30 40 50"><geom type="box" size="0.1 0.2 0.3" quat="1 2 3 4"/></frame>"#,
        [
            0xbfdbcf7357d877e0,
            0x3fcefa04c5cd8dac,
            0x3fd7d592391721d2,
            0x3fe912b10fddb0ac,
        ],
    ),
    // frame_nested
    (
        "frame_nested",
        r#"<frame euler="30 40 50"><frame quat="1 2 3 4"><geom type="box" size="0.1 0.2 0.3" axisangle="1 2 3 45"/></frame></frame>"#,
        [
            0xbfe7c502c8c1855f,
            0x3fc10a830f5dc365,
            0x3fd0b531783543cb,
            0x3fe3435f8aa38613,
        ],
    ),
];

#[test]
fn a_frames_turn_is_composed_with_its_geoms() {
    for &(what, frame, bits) in FRAME_CASES {
        let model = load_model(&format!(
            r#"<mujoco><worldbody><body name="b"><joint type="hinge"/><geom type="sphere" size="0.1"/>{frame}</body></worldbody></mujoco>"#
        ))
        .unwrap();
        let q = model.geom_quat[1].into_inner();
        let got = [q.w, q.i, q.j, q.k];
        let mujoco = bits.map(f64::from_bits);
        assert!(
            is_mujocos(got, mujoco, Some(1e-14)),
            "{what}: {got:?}, MuJoCo {mujoco:?}"
        );
    }
}

/// Each is refused by MuJoCo 3.5.0, which compiles a body before fusestatic
/// moves it and its contents.
const MOVED_REFUSED: &[(&str, &str, &str, &str)] = &[
    // fused_refuse_axisangle
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p"><joint type="hinge"/><geom type="sphere" size="0.1"/><body name="b" axisangle="0 0 0 30"><geom type="sphere" size="0.1"/></body></body></worldbody></mujoco>"#,
        "body",
        "axisangle",
        "axisangle too small",
    ),
    // fused_geom_refuse_zaxis
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p"><joint type="hinge"/><geom type="sphere" size="0.1"/><body name="b" euler="0 0 30"><geom type="sphere" size="0.1" zaxis="0 0 0"/></body></body></worldbody></mujoco>"#,
        "geom",
        "zaxis",
        "zaxis too small",
    ),
    // fused_child_refuse_xyaxes
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p"><joint type="hinge"/><geom type="sphere" size="0.1"/><body name="b" euler="0 0 30"><body name="c" xyaxes="0 0 0 0 1 0"><joint type="hinge"/><geom type="sphere" size="0.1"/></body></body></body></worldbody></mujoco>"#,
        "body",
        "xyaxes",
        "xaxis too small",
    ),
    // body_in_frame_refuse_zaxis
    (
        r#"<mujoco><worldbody><frame euler="0 0 30"><body name="b" zaxis="0 0 0"><joint type="hinge"/><geom type="sphere" size="0.1"/></body></frame></worldbody></mujoco>"#,
        "body",
        "zaxis",
        "zaxis too small",
    ),
    // site_in_frame_refuse_axisangle
    (
        r#"<mujoco><worldbody><body name="b"><joint type="hinge"/><geom type="sphere" size="0.1"/><frame euler="0 0 30"><site name="s" axisangle="0 0 0 30"/></frame></body></worldbody></mujoco>"#,
        "site",
        "axisangle",
        "axisangle too small",
    ),
    // geom_in_frame_refuse_yaxis
    (
        r#"<mujoco><worldbody><body name="b"><joint type="hinge"/><geom type="sphere" size="0.1"/><frame euler="0 0 30"><geom type="box" size="0.1 0.2 0.3" xyaxes="1 0 0 2 0 0"/></frame></body></worldbody></mujoco>"#,
        "geom",
        "xyaxes",
        "yaxis too small",
    ),
    // fused_site_refuse_zaxis
    (
        r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p"><joint type="hinge"/><geom type="sphere" size="0.1"/><body name="b" euler="0 0 30"><site name="s" zaxis="0 0 0"/></body></body></worldbody></mujoco>"#,
        "site",
        "zaxis",
        "zaxis too small",
    ),
];

#[test]
fn an_alternative_fusestatic_or_a_frame_moves_is_refused_too() {
    for &(xml, tag, attribute, reason) in MOVED_REFUSED {
        let err = load_model(xml).unwrap_err();
        assert!(
            matches!(&err, MjcfError::InvalidValue { element: e, attribute: a, reason: r, .. }
                if e == tag && a == attribute && r == reason),
            "{xml}: {err}"
        );
    }
}

/// MuJoCo 3.5.0 turns a geom by its `fromto` and never resolves its alternative
/// (`mjCGeom::Compile`, `user_objects.cc:3677-3724`), so a too-small one loads.
#[test]
fn a_fromto_geoms_alternative_is_not_resolved() {
    load_model(FROMTO_ZAXIS).unwrap();
    load_model(FUSED_FROMTO_ZAXIS).unwrap();
}

const FROMTO_ZAXIS: &str = r#"<mujoco><worldbody><body name="b"><joint type="hinge"/><geom type="capsule" size="0.1" fromto="0 0 0 0 0 1" zaxis="0 0 0"/></body></worldbody></mujoco>"#;
const FUSED_FROMTO_ZAXIS: &str = r#"<mujoco><compiler fusestatic="true"/><worldbody><body name="p"><joint type="hinge"/><geom type="sphere" size="0.1"/><body name="b" euler="0 0 30"><geom type="capsule" size="0.1" fromto="0 0 0 0 0 1" zaxis="0 0 0"/></body></body></worldbody></mujoco>"#;
