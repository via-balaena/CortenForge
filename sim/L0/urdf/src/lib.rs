//! URDF robot description parser for physics simulation.
//!
//! This crate parses [URDF](http://wiki.ros.org/urdf) (Unified Robot Description Format)
//! files and converts them into simulation-ready types for use with `sim-core`.
//!
//! # Features
//!
//! - Parse URDF XML from files or strings
//! - Convert to MuJoCo-aligned Model/Data architecture
//! - Support for box, sphere, cylinder, and mesh collision geometry
//! - Kinematic tree validation
//!
//! # Layer 0
//!
//! This is a Layer 0 crate with **zero Bevy dependencies**. It can be used in:
//!
//! - Headless training environments
//! - Hardware control systems
//! - Analysis tools
//! - Other game engines
//!
//! # Example
//!
//! ```
//! use sim_urdf::load_urdf_model;
//!
//! // Load a simple robot from URDF
//! let urdf = r#"
//!     <robot name="simple">
//!         <link name="base_link">
//!             <inertial>
//!                 <mass value="1.0"/>
//!                 <inertia ixx="0.1" iyy="0.1" izz="0.1"/>
//!             </inertial>
//!         </link>
//!     </robot>
//! "#;
//!
//! let model = load_urdf_model(urdf).expect("should parse");
//! assert_eq!(model.name, "simple");
//!
//! // Create simulation data and step
//! let mut data = model.make_data();
//! for _ in 0..100 {
//!     data.step(&model).expect("step failed");
//! }
//! ```
//!
//! # Supported URDF Elements
//!
//! ## Links
//!
//! - `<link name="...">` - Rigid body definition
//! - `<inertial>` - Mass, center of mass, inertia tensor
//! - `<collision>` - Collision geometry (box, sphere, cylinder, mesh)
//! - `<visual>` - Converted to geometry, then discarded by the MJCF compiler
//!   (`discardvisual="true"`; Layer 0 has no rendering)
//!
//! ## Joints
//!
//! - `<joint name="..." type="...">` - Joint constraint
//! - Supported types: `fixed`, `revolute`, `continuous`, `prismatic`, `floating`, `planar`
//! - `<parent>`, `<child>` - Connected links
//! - `<origin>` - Joint frame relative to parent
//! - `<axis>` - Joint axis for revolute/prismatic
//! - `<limit>` - Position, velocity, effort limits
//! - `<dynamics>` - Damping and friction
//!
//! ## Geometry
//!
//! - `<box size="x y z"/>` - Box collision shape
//! - `<sphere radius="r"/>` - Sphere collision shape
//! - `<cylinder radius="r" length="l"/>` - Native cylinder collision shape
//! - `<mesh filename="..."/>` - Emitted as an MJCF `<mesh>` asset + geom; the
//!   mesh file itself is resolved by the MJCF compiler, not loaded here
//!
//! # Limitations
//!
//! - `<gazebo>` extensions are ignored
//! - Kinematic loops are not supported (tree structures only)
//!
//! # Coordinate System
//!
//! URDF uses a right-handed coordinate system, and `MuJoCo`/MJCF shares the
//! same right-handed, Z-up convention. Link and joint frames (`xyz`, `rpy`,
//! `axis`) are therefore passed through verbatim, with no axis remapping.

#![deny(clippy::unwrap_used, clippy::expect_used)]
#![warn(missing_docs)]
#![allow(
    clippy::missing_const_for_fn,
    clippy::module_name_repetitions,
    clippy::unnested_or_patterns,
    clippy::similar_names,
    clippy::must_use_candidate,
    clippy::missing_errors_doc,
    clippy::missing_panics_doc,
    clippy::unnecessary_wraps,
    clippy::redundant_closure_for_method_calls,
    clippy::should_implement_trait,
    clippy::items_after_statements,
    clippy::unnecessary_lazy_evaluations,
    clippy::needless_pass_by_value,
    clippy::map_unwrap_or,
    clippy::option_if_let_else,
    clippy::unused_self,
    clippy::redundant_pattern_matching
)]

mod converter;
mod error;
mod parser;
mod types;
mod validation;

// Re-export main types
pub use converter::{robot_to_mjcf, urdf_to_mjcf};
pub use error::{Result, UrdfError};
pub use parser::parse_urdf_str;
pub use types::{
    UrdfCollision, UrdfGeometry, UrdfInertia, UrdfInertial, UrdfJoint, UrdfJointDynamics,
    UrdfJointLimit, UrdfJointType, UrdfLink, UrdfMimic, UrdfOrigin, UrdfRobot, UrdfVisual,
};
pub use validation::{ValidationResult, validate};

/// Load a URDF file and convert it directly to a MuJoCo-aligned Model.
///
/// This is the recommended way to load URDF robots for simulation. It:
/// 1. Parses the URDF XML
/// 2. Converts to MJCF XML (single compilation path)
/// 3. Loads via `sim_mjcf::load_model()`
///
/// # Example
///
/// ```
/// use sim_urdf::load_urdf_model;
///
/// let urdf = r#"
///     <robot name="arm">
///         <link name="base">
///             <inertial>
///                 <mass value="1.0"/>
///                 <inertia ixx="0.1" iyy="0.1" izz="0.1"/>
///             </inertial>
///         </link>
///     </robot>
/// "#;
/// let model = load_urdf_model(urdf).expect("should load");
/// let mut data = model.make_data();
/// data.step(&model).expect("step failed");
/// ```
pub fn load_urdf_model(urdf_xml: &str) -> Result<sim_core::Model> {
    let mjcf = urdf_to_mjcf(urdf_xml)?;
    sim_mjcf::load_model(&mjcf)
        .map_err(|e| UrdfError::Unsupported(format!("MJCF conversion error: {e}")))
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    /// Test Model/Data API with a two-link arm.
    #[test]
    fn test_two_link_arm_model_data() {
        let urdf = r#"
            <robot name="two_link_arm">
                <link name="base_link">
                    <inertial>
                        <mass value="5.0"/>
                        <inertia ixx="0.5" iyy="0.5" izz="0.5"/>
                    </inertial>
                </link>

                <link name="link1">
                    <inertial>
                        <origin xyz="0 0 0.25"/>
                        <mass value="1.0"/>
                        <inertia ixx="0.1" iyy="0.1" izz="0.01"/>
                    </inertial>
                    <collision>
                        <origin xyz="0 0 0.25"/>
                        <geometry>
                            <cylinder radius="0.05" length="0.5"/>
                        </geometry>
                    </collision>
                </link>

                <link name="link2">
                    <inertial>
                        <origin xyz="0 0 0.2"/>
                        <mass value="0.5"/>
                        <inertia ixx="0.05" iyy="0.05" izz="0.005"/>
                    </inertial>
                    <collision>
                        <origin xyz="0 0 0.2"/>
                        <geometry>
                            <sphere radius="0.08"/>
                        </geometry>
                    </collision>
                </link>

                <joint name="joint1" type="revolute">
                    <parent link="base_link"/>
                    <child link="link1"/>
                    <origin xyz="0 0 0.1"/>
                    <axis xyz="0 1 0"/>
                    <limit lower="-3.14" upper="3.14" effort="100" velocity="2"/>
                    <dynamics damping="0.5" friction="0.1"/>
                </joint>

                <joint name="joint2" type="revolute">
                    <parent link="link1"/>
                    <child link="link2"/>
                    <origin xyz="0 0 0.5"/>
                    <axis xyz="0 1 0"/>
                    <limit lower="-2.0" upper="2.0" effort="50" velocity="3"/>
                    <dynamics damping="0.2"/>
                </joint>
            </robot>
        "#;

        // Load into Model
        let model = load_urdf_model(urdf).expect("should load");
        assert_eq!(model.name, "two_link_arm");

        // Create Data and step
        let mut data = model.make_data();
        data.forward(&model).expect("forward failed");

        // Verify structure
        assert!(model.nbody >= 3, "Should have at least 3 bodies");
        assert_eq!(model.njnt, 2, "Should have 2 joints");
    }

    /// Test error handling for invalid URDF.
    #[test]
    fn test_invalid_urdf() {
        // Missing robot element
        let result = parse_urdf_str("<link name='test'/>");
        assert!(result.is_err());

        // Invalid joint type
        let result = parse_urdf_str(
            r#"
            <robot name="test">
                <link name="a"/>
                <link name="b"/>
                <joint name="j" type="invalid">
                    <parent link="a"/>
                    <child link="b"/>
                </joint>
            </robot>
        "#,
        );
        assert!(matches!(result, Err(UrdfError::UnknownJointType(_))));
    }

    /// Test validation catches kinematic issues.
    #[test]
    fn test_validation_errors() {
        // Undefined link reference
        let robot = UrdfRobot::new("test")
            .with_link(UrdfLink::new("base"))
            .with_joint(UrdfJoint::new(
                "j1",
                UrdfJointType::Fixed,
                "base",
                "nonexistent",
            ));

        let result = validate(&robot);
        assert!(matches!(result, Err(UrdfError::UndefinedLink { .. })));
    }

    /// Test simple robot with Model/Data API.
    #[test]
    fn test_simple_robot_model_data() {
        let urdf = r#"
            <robot name="simple">
                <link name="base_link">
                    <inertial>
                        <mass value="1.0"/>
                        <inertia ixx="0.1" iyy="0.1" izz="0.1"/>
                    </inertial>
                </link>
            </robot>
        "#;

        let model = load_urdf_model(urdf).expect("should load");
        assert_eq!(model.name, "simple");

        let mut data = model.make_data();
        data.forward(&model).expect("forward failed");

        // Step a few times
        for _ in 0..10 {
            data.step(&model).expect("step failed");
        }
    }

    /// A chain of `n` links, each joined to the one before by a revolute
    /// joint.
    fn chain(n: usize) -> String {
        use std::fmt::Write as _;
        let mut urdf = String::from(r#"<robot name="chain">"#);
        for k in 0..n {
            write!(
                urdf,
                r#"<link name="l{k}"><inertial><mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/></inertial></link>"#
            )
            .unwrap();
        }
        for k in 1..n {
            write!(
                urdf,
                r#"<joint name="j{k}" type="revolute"><parent link="l{}"/><child link="l{k}"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>"#,
                k - 1
            )
            .unwrap();
        }
        urdf.push_str("</robot>");
        urdf
    }

    /// A robot of `links`, each with a unit inertia, and revolute `joints`,
    /// each a name, a parent link and a child link.
    fn robot(links: &[&str], joints: &[(&str, &str, &str)]) -> String {
        use std::fmt::Write as _;
        let mut urdf = String::from(r#"<robot name="r">"#);
        for n in links {
            write!(
                urdf,
                r#"<link name="{n}"><inertial><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link>"#
            )
            .unwrap();
        }
        for (n, parent, child) in joints {
            write!(
                urdf,
                r#"<joint name="{n}" type="revolute"><parent link="{parent}"/><child link="{child}"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>"#
            )
            .unwrap();
        }
        urdf.push_str("</robot>");
        urdf
    }

    /// A two-link robot whose one revolute (or prismatic) joint has `limit`.
    fn limited_pair(joint_type: &str, limit: &str) -> String {
        format!(
            r#"<robot name="r">
                <link name="a"><inertial><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link>
                <link name="b"><inertial><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link>
                <joint name="j" type="{joint_type}"><parent link="a"/><child link="b"/><axis xyz="0 0 1"/>{limit}</joint>
            </robot>"#
        )
    }

    /// Each case's joint limit and range as MuJoCo 3.5.0's URDF importer
    /// builds them (measured): one bound written, a lower above the upper, or
    /// equal bounds leave the joint unlimited; a bound not written is 0.
    #[test]
    fn a_urdf_limit_is_read_as_mujocos_importer_reads_it() {
        let cases = [
            ("revolute", r#"lower="-1" upper="1""#, true, (-1.0, 1.0)),
            ("revolute", r#"upper="1""#, false, (0.0, 1.0)),
            ("prismatic", r#"lower="-0.2""#, false, (-0.2, 0.0)),
            ("revolute", r#"lower="1" upper="0.5""#, false, (1.0, 0.5)),
            ("revolute", r#"lower="0.5" upper="0.5""#, false, (0.5, 0.5)),
            ("revolute", "", false, (0.0, 0.0)),
        ];
        for (joint_type, bounds, limited, range) in cases {
            let limit = format!(r#"<limit {bounds} effort="1" velocity="1"/>"#);
            let model = load_urdf_model(&limited_pair(joint_type, &limit))
                .map_err(|e| format!("{joint_type} {bounds}: {e}"))
                .expect("the robot loads");
            assert_eq!(model.jnt_limited[0], limited, "{joint_type} {bounds}");
            assert_eq!(model.jnt_range[0], range, "{joint_type} {bounds}");
        }
    }

    /// A link's children are converted depth-first in the order their joints
    /// are written, as MuJoCo 3.5.0's URDF importer orders its bodies
    /// (measured: joints j1 base→c1, j2 base→c2, j3 c1→g1 give c1, g1, c2).
    #[test]
    fn a_links_children_follow_their_joints_order() {
        let urdf = robot(
            &["base", "c1", "c2", "g1"],
            &[
                ("j1", "base", "c1"),
                ("j2", "base", "c2"),
                ("j3", "c1", "g1"),
            ],
        );
        let model = load_urdf_model(&urdf).expect("loads");
        let order: Vec<&str> = model
            .body_name
            .iter()
            .filter_map(|n| n.as_deref())
            .filter(|n| ["c1", "c2", "g1"].contains(n))
            .collect();
        assert_eq!(order, ["c1", "g1", "c2"]);
    }

    /// A link with a second parent joint is refused, as MuJoCo 3.5.0's
    /// importer refuses it ("URDF body has multiple parents", measured),
    /// whether or not the joints close a loop; a loop no root reaches is
    /// refused too ("URDF body not found", measured). The case without a loop
    /// runs first: were the refusal lost, the walk around the third case's
    /// loop would not end.
    #[test]
    fn a_second_parent_joint_or_a_loop_no_root_reaches_is_refused() {
        let cases = [
            (
                robot(
                    &["r", "a", "b"],
                    &[("j1", "r", "a"), ("j2", "r", "b"), ("j3", "a", "b")],
                ),
                "link 'b' has multiple parent joints",
            ),
            (
                robot(&["r", "a"], &[("j1", "r", "a"), ("j2", "a", "a")]),
                "link 'a' has multiple parent joints",
            ),
            (
                robot(
                    &["r", "a", "b"],
                    &[("j1", "r", "a"), ("j2", "a", "b"), ("j3", "b", "a")],
                ),
                "link 'a' has multiple parent joints",
            ),
            (
                robot(&["r", "c", "d"], &[("j1", "c", "d"), ("j2", "d", "c")]),
                "link 'c' is reached from no root link",
            ),
        ];
        for (urdf, message) in cases {
            let result = urdf_to_mjcf(&urdf);
            assert!(
                matches!(&result, Err(UrdfError::KinematicLoop(m)) if m.contains(message)),
                "{message}: {result:?}"
            );
        }
    }

    /// A joint naming a link the robot has not, and a link name given twice,
    /// are refused, as MuJoCo 3.5.0's importer refuses them (measured: "URDF
    /// joint parent or child missing", "repeated URDF name").
    #[test]
    fn a_joint_naming_no_link_or_a_repeated_link_name_is_refused() {
        for (urdf, missing) in [
            (robot(&["r", "a"], &[("j1", "ghost", "a")]), "ghost"),
            (robot(&["r"], &[("j1", "r", "ghost")]), "ghost"),
        ] {
            let result = urdf_to_mjcf(&urdf);
            assert!(
                matches!(&result, Err(UrdfError::UndefinedLink { link_name, joint_name })
                    if link_name == missing && joint_name == "j1"),
                "{result:?}"
            );
        }
        for (urdf, repeated) in [
            (robot(&["r", "r"], &[]), "r"),
            (robot(&["r", "a", "a"], &[("j1", "r", "a")]), "a"),
        ] {
            let result = urdf_to_mjcf(&urdf);
            assert!(
                matches!(&result, Err(UrdfError::DuplicateLink(name)) if name == repeated),
                "{result:?}"
            );
        }
    }

    /// Every link no joint enters is a root, its tree under the world, as
    /// MuJoCo 3.5.0's importer writes them (measured: roots r and s, s with a
    /// child t, give nbody 2 and one joint, the jointless roots fused into
    /// the world).
    #[test]
    fn every_link_no_joint_enters_is_a_root() {
        let model = load_urdf_model(&robot(&["r", "s", "t"], &[("j1", "s", "t")])).expect("loads");
        assert_eq!((model.nbody, model.njnt), (2, 1));
        assert_eq!(model.body_name[1].as_deref(), Some("t"));
    }

    /// A planar joint as MuJoCo 3.5.0's URDF importer builds it (measured):
    /// slides `_TX` and `_TY`, then a hinge `_RZ`, on MuJoCo's axes; the limit
    /// on `_TX` alone, by revolute's rule; the dynamics on all three.
    #[test]
    fn a_planar_joint_is_built_as_mujocos_importer_builds_it() {
        let planar = |axis: &str, extra: &str| {
            format!(
                r#"<robot name="r">
                    <link name="a"><inertial><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link>
                    <link name="b"><inertial><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link>
                    <joint name="p" type="planar"><parent link="a"/><child link="b"/><axis xyz="{axis}"/>{extra}</joint>
                </robot>"#
            )
        };
        // Each axis with MuJoCo's three joint axes for it.
        let cases = [
            ("0 0 1", [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
            (
                "1 0 0",
                [
                    [2.220_446_049_250_313e-16, 0.0, -1.0],
                    [0.0, 1.0, 0.0],
                    [1.0, 0.0, 0.0],
                ],
            ),
            (
                "0 0.6 0.8",
                [
                    [0.999_999_999_999_999_9, 0.0, 0.0],
                    [0.0, 0.799_999_999_999_999_9, -0.6],
                    [0.0, 0.6, 0.8],
                ],
            ),
            (
                "0.3 -0.4 0.5",
                [
                    [
                        0.894_558_441_227_157_1,
                        0.140_588_745_030_457_2,
                        -0.424_264_068_711_928_5,
                    ],
                    [
                        0.140_588_745_030_457_2,
                        0.812_548_339_959_390_4,
                        0.565_685_424_949_238_1,
                    ],
                    [
                        0.424_264_068_711_928_45,
                        -0.565_685_424_949_238,
                        0.707_106_781_186_547_5,
                    ],
                ],
            ),
            (
                "0 0 -1",
                [
                    [1.0, 0.0, 0.0],
                    [0.0, -1.0, 1.224_646_799_147_353_2e-16],
                    [0.0, 0.0, -1.0],
                ],
            ),
            (
                "1e-8 0 1",
                [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1e-8, 0.0, 1.0]],
            ),
        ];
        let extra = r#"<limit lower="-1" upper="2" effort="1" velocity="1"/><dynamics damping="0.5" friction="0.25"/>"#;
        for (axis, expected) in cases {
            let model = load_urdf_model(&planar(axis, extra)).expect("loads");
            let names: Vec<&str> = model.jnt_name.iter().filter_map(|n| n.as_deref()).collect();
            assert_eq!(names, ["p_TX", "p_TY", "p_RZ"], "{axis}");
            for (k, want) in expected.iter().enumerate() {
                let got = model.jnt_axis[k];
                let gap = (0..3).map(|i| (got[i] - want[i]).abs()).fold(0.0, f64::max);
                assert!(
                    gap <= 1e-15,
                    "{axis}: joint {k} axis {got:?}, MuJoCo {want:?}"
                );
            }
            assert_eq!(model.jnt_limited, [true, false, false], "{axis}");
            assert_eq!(model.jnt_range[0], (-1.0, 2.0), "{axis}");
            assert_eq!(model.dof_damping, [0.5; 3], "{axis}");
            assert_eq!(model.dof_frictionloss, [0.25; 3], "{axis}");
        }
        for bounds in [r#"upper="1""#, r#"lower="1" upper="0.5""#] {
            let limit = format!(r#"<limit {bounds} effort="1" velocity="1"/>"#);
            let model = load_urdf_model(&planar("0 0 1", &limit)).expect("loads");
            assert_eq!(model.jnt_limited, [false; 3], "{bounds}");
        }
    }

    /// The converter nests a body per link without recursing, so a chain far
    /// past the XML depth rule is refused by it, not by the caller's stack
    /// (ledger L303: 5000 links overflowed a 2-mebibyte thread).
    #[test]
    fn a_long_chain_is_refused_without_overflowing_the_callers_stack() {
        let urdf = chain(5000);
        let err = std::thread::Builder::new()
            .stack_size(2 << 20)
            .spawn(move || load_urdf_model(&urdf).map(|_| ()))
            .unwrap()
            .join()
            .expect("the converter does not overflow a 2 MiB stack")
            .unwrap_err();
        assert!(err.to_string().contains("nested deeper than 498"), "{err}");
    }

    /// The conversion nests one body per link, so the XML depth rule sim-mjcf
    /// applies (MuJoCo's XML reader's) bounds a chain: 496 links load, 497 are
    /// refused. MuJoCo 3.5.0 loads URDF chains of 497 and 600 links (registry
    /// row D-URDF-DEPTH).
    #[test]
    fn a_chain_of_more_than_496_links_is_refused_by_the_xml_depth_rule() {
        assert!(load_urdf_model(&chain(496)).is_ok());
        let err = load_urdf_model(&chain(497)).unwrap_err();
        assert!(err.to_string().contains("nested deeper than 498"), "{err}");
    }
}
