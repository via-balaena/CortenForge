//! Factory methods for common mechanical systems.
//!
//! These constructors produce pre-configured [`Model`] instances for
//! canonical test systems (pendulums, free bodies, etc.). Used by
//! inline tests and by `sim-conformance-tests`.

use nalgebra::{DVector, UnitQuaternion, Vector3};
use std::f64::consts::PI;

use super::enums::{GeomType, MjJointType, SameFrame};
use super::model::Model;
use super::model_derived::local_aabb_from_primitive;
use crate::constraint::impedance::{DEFAULT_SOLIMP, DEFAULT_SOLREF};

impl Model {
    /// Create an n-link serial pendulum (hinge joints only).
    ///
    /// This creates a serial chain of `n` bodies connected by hinge joints,
    /// all rotating around the Y axis. Each body has a point mass at its end.
    ///
    /// # Arguments
    /// * `n` - Number of links (must be >= 1)
    /// * `link_length` - Length of each link (meters)
    /// * `link_mass` - Mass of each link (kg)
    ///
    /// # Returns
    /// A `Model` representing the n-link pendulum with all joints at qpos=0
    /// (hanging straight down).
    ///
    /// # Panics
    /// Panics if `n` is 0 (requires at least 1 link).
    ///
    /// # Example
    /// ```ignore
    /// let model = Model::n_link_pendulum(3, 1.0, 1.0);
    /// let mut data = model.make_data();
    /// data.qpos[0] = std::f64::consts::PI / 4.0; // Tilt first link
    /// data.forward(&model);
    /// ```
    #[must_use]
    pub fn n_link_pendulum(n: usize, link_length: f64, link_mass: f64) -> Self {
        assert!(n >= 1, "n_link_pendulum requires at least 1 link");

        let mut model = Self::empty();

        // Dimensions
        model.nq = n;
        model.nv = n;
        model.nbody = n + 1; // world + n bodies
        model.njnt = n;

        // Build the kinematic chain
        for i in 0..n {
            let body_id = i + 1; // Body 0 is world
            let parent_id = i; // Each body's parent is the previous body (0 = world for first)

            // Body tree
            model.body_parent.push(parent_id);
            model.body_rootid.push(1); // All belong to tree rooted at body 1
            model.body_jnt_adr.push(i);
            model.body_jnt_num.push(1);
            model.body_dof_adr.push(i);
            model.body_dof_num.push(1);
            model.body_geom_adr.push(0);
            model.body_geom_num.push(0);

            // Body properties — MuJoCo convention:
            //
            // Body 1: body_pos = (0,0,0) — body frame at parent (world) origin,
            //          coinciding with the joint. Only orientation changes with qpos.
            // Body 2+: body_pos = (0,0,-L) — body frame at end of parent link,
            //           which is where this body's joint is.
            //
            // For all bodies: body_ipos = (0,0,-L) — COM (point mass) at end of
            // the link, below the body frame. xipos swings; xpos stays at the joint.
            //
            // Visualization should use xipos (COM position), not xpos (joint position).
            let body_offset = if i == 0 {
                Vector3::zeros() // Body 1: frame at parent origin (joint location)
            } else {
                Vector3::new(0.0, 0.0, -link_length) // Body 2+: frame at parent's link end
            };
            model.body_pos.push(body_offset);
            model.body_quat.push(UnitQuaternion::identity());
            model.body_ipos.push(Vector3::new(0.0, 0.0, -link_length)); // COM at end of link
            model.body_iquat.push(UnitQuaternion::identity());
            model.body_mass.push(link_mass);
            // Thin rod inertia about COM: I_xx = I_yy = m*L²/12, I_zz ≈ small
            // (rod along Z-axis, COM at end of link via body_ipos offset)
            let i_transverse = link_mass * link_length * link_length / 12.0;
            let i_axial = 0.01 * i_transverse; // Thin rod approximation
            model
                .body_inertia
                .push(Vector3::new(i_transverse, i_transverse, i_axial));
            model.body_name.push(Some(format!("link_{i}")));
            model.body_subtreemass.push(0.0); // Will be computed after model is built
            model.body_mocapid.push(None);

            // Joint definition (hinge rotating around Y axis).
            // jnt_pos = (0,0,0) — joint at body frame origin (MuJoCo convention).
            model.jnt_type.push(MjJointType::Hinge);
            model.jnt_body.push(body_id);
            model.jnt_qpos_adr.push(i);
            model.jnt_dof_adr.push(i);
            model.jnt_pos.push(Vector3::zeros()); // Joint at body frame origin
            model.jnt_axis.push(Vector3::new(0.0, 1.0, 0.0)); // Rotate around Y
            model.jnt_limited.push(false);
            model.jnt_range.push((-PI, PI));
            model.jnt_stiffness.push(0.0);
            model.jnt_springref.push(0.0);
            model.jnt_damping.push(0.0);
            model.jnt_solref.push(DEFAULT_SOLREF);
            model.jnt_solimp.push(DEFAULT_SOLIMP);
            model.jnt_name.push(Some(format!("hinge_{i}")));
            model.qpos_spring.push(0.0); // Hinge: scalar springref

            // DOF definition
            model.dof_body.push(body_id);
            model.dof_jnt.push(i);
            model
                .dof_parent
                .push(if i == 0 { None } else { Some(i - 1) });
            model.dof_armature.push(0.0);
            model.dof_damping.push(0.0);
            model.dof_frictionloss.push(0.0);
        }

        // Default qpos (hanging down)
        model.qpos0 = DVector::zeros(n);

        // Physics options
        model.timestep = 1.0 / 240.0;
        model.gravity = Vector3::new(0.0, 0.0, -9.81);
        model.solver_iterations = 10;
        model.solver_tolerance = 1e-8;

        // Per-DOF constraint-solver parameters, so a friction-loss or joint-limit
        // constraint assembles correctly if one is added (the MJCF builder populates
        // these for every DOF; the factory left them empty, which panicked on the
        // first `dof_solref`/`dof_solimp` read and mis-scaled the constraint via the
        // `MJ_MINVAL` invweight fallback). Inert for the default constraint-free use.
        model.dof_solref = vec![DEFAULT_SOLREF; model.nv];
        model.dof_solimp = vec![DEFAULT_SOLIMP; model.nv];

        recompute(&mut model);
        model
    }

    /// Create a double pendulum (2-link serial chain).
    ///
    /// Convenience method equivalent to `Model::n_link_pendulum(2, ...)`.
    #[must_use]
    pub fn double_pendulum(link_length: f64, link_mass: f64) -> Self {
        Self::n_link_pendulum(2, link_length, link_mass)
    }

    /// Create a single body carrying THREE scalar joints (hinge → slide →
    /// hinge) with non-parallel axes at offset pivots.
    ///
    /// This is the canonical fixture for the multi-joint partial-frame motion
    /// subspace: an earlier joint's `cdof` axis/anchor must be captured BEFORE
    /// the later same-body joints rotate the body frame. Here the first hinge
    /// and the slide are each followed by a later *rotating* joint (the second
    /// hinge), so a final-frame `cdof` computation over-rotates their subspace,
    /// while the partial-frame capture is correct. `nq = nv = 3`.
    ///
    /// The body is off-COM (`body_ipos ≠ 0`) so the angular/linear coupling is
    /// material, and the joint pivots are offset (`jnt_pos ≠ 0`) so the hinge
    /// `axis × r` lever is non-trivial.
    #[must_use]
    pub fn multi_joint_body() -> Self {
        let mut model = Self::empty();

        // Dimensions: one body, three scalar DOFs.
        model.nq = 3;
        model.nv = 3;
        model.nbody = 2; // world + body
        model.njnt = 3;

        // Body 1
        model.body_parent.push(0);
        model.body_rootid.push(1);
        model.body_jnt_adr.push(0);
        model.body_jnt_num.push(3);
        model.body_dof_adr.push(0);
        model.body_dof_num.push(3);
        model.body_geom_adr.push(0);
        model.body_geom_num.push(0);
        model.body_pos.push(Vector3::new(0.1, -0.05, 0.2));
        model.body_quat.push(UnitQuaternion::identity());
        model.body_ipos.push(Vector3::new(0.05, 0.1, -0.15)); // off-COM
        model.body_iquat.push(UnitQuaternion::identity());
        model.body_mass.push(1.3);
        model.body_inertia.push(Vector3::new(0.02, 0.03, 0.04));
        model.body_name.push(Some("multi".to_string()));
        model.body_subtreemass.push(0.0);
        model.body_mocapid.push(None);

        // Three joints: hinge (j0), slide (j1), hinge (j2), non-parallel axes,
        // offset pivots.
        let jnt_specs = [
            (
                MjJointType::Hinge,
                Vector3::new(1.0, 0.3, 0.0).normalize(),
                Vector3::new(0.0, 0.02, -0.03),
                "h0",
            ),
            (
                MjJointType::Slide,
                Vector3::new(0.2, 1.0, 0.1).normalize(),
                Vector3::new(0.01, 0.0, 0.04),
                "s1",
            ),
            (
                MjJointType::Hinge,
                Vector3::new(0.0, 0.2, 1.0).normalize(),
                Vector3::new(-0.02, 0.03, 0.0),
                "h2",
            ),
        ];
        for (i, (jtype, axis, pos, name)) in jnt_specs.into_iter().enumerate() {
            model.jnt_type.push(jtype);
            model.jnt_body.push(1);
            model.jnt_qpos_adr.push(i);
            model.jnt_dof_adr.push(i);
            model.jnt_pos.push(pos);
            model.jnt_axis.push(axis);
            model.jnt_limited.push(false);
            model.jnt_range.push((-PI, PI));
            model.jnt_stiffness.push(0.0);
            model.jnt_springref.push(0.0);
            model.jnt_damping.push(0.0);
            model.jnt_solref.push(DEFAULT_SOLREF);
            model.jnt_solimp.push(DEFAULT_SOLIMP);
            model.jnt_name.push(Some(name.to_string()));
            model.qpos_spring.push(0.0);

            model.dof_body.push(1);
            model.dof_jnt.push(i);
            model
                .dof_parent
                .push(if i == 0 { None } else { Some(i - 1) });
            model.dof_armature.push(0.0);
            model.dof_damping.push(0.0);
            model.dof_frictionloss.push(0.0);
        }

        model.qpos0 = DVector::zeros(3);

        model.timestep = 1.0 / 240.0;
        model.gravity = Vector3::new(0.0, 0.0, -9.81);
        model.solver_iterations = 10;
        model.solver_tolerance = 1e-8;

        recompute(&mut model);
        model
    }

    /// Create a spherical pendulum (ball joint at origin).
    ///
    /// This creates a single body attached to the world by a ball joint,
    /// allowing 3-DOF rotation. The body has a point mass at distance `length`
    /// below the joint.
    ///
    /// # Arguments
    /// * `length` - Length from pivot to mass (meters)
    /// * `mass` - Point mass (kg)
    ///
    /// # Note
    /// The ball joint uses quaternion representation (nq=4, nv=3).
    /// Initial state is qpos=`[1,0,0,0]` (identity quaternion = hanging down).
    #[must_use]
    pub fn spherical_pendulum(length: f64, mass: f64) -> Self {
        let mut model = Self::empty();

        // Dimensions
        model.nq = 4; // Quaternion: w, x, y, z
        model.nv = 3; // Angular velocity: omega_x, omega_y, omega_z
        model.nbody = 2; // world + pendulum
        model.njnt = 1;

        // Body 1: pendulum bob
        model.body_parent.push(0);
        model.body_rootid.push(1);
        model.body_jnt_adr.push(0);
        model.body_jnt_num.push(1);
        model.body_dof_adr.push(0);
        model.body_dof_num.push(3);
        model.body_geom_adr.push(0);
        model.body_geom_num.push(0);

        // MuJoCo convention: body frame at joint (parent origin), COM offset below.
        model.body_pos.push(Vector3::zeros()); // Body frame at parent = joint location
        model.body_quat.push(UnitQuaternion::identity());
        model.body_ipos.push(Vector3::new(0.0, 0.0, -length)); // COM at end of pendulum
        model.body_iquat.push(UnitQuaternion::identity());
        model.body_mass.push(mass);
        // Thin rod inertia about COM
        let i_transverse = mass * length * length / 12.0;
        let i_axial = 0.01 * i_transverse;
        model
            .body_inertia
            .push(Vector3::new(i_transverse, i_transverse, i_axial));
        model.body_name.push(Some("bob".to_string()));
        model.body_subtreemass.push(0.0); // Will be computed after model is built
        model.body_mocapid.push(None);

        // Ball joint at world origin
        model.jnt_type.push(MjJointType::Ball);
        model.jnt_body.push(1);
        model.jnt_qpos_adr.push(0);
        model.jnt_dof_adr.push(0);
        model.jnt_pos.push(Vector3::zeros());
        model.jnt_axis.push(Vector3::z()); // Not used for ball, but required
        model.jnt_limited.push(false);
        model.jnt_range.push((-PI, PI));
        model.jnt_stiffness.push(0.0);
        model.jnt_springref.push(0.0);
        model.jnt_damping.push(0.0);
        model.jnt_solref.push(DEFAULT_SOLREF);
        model.jnt_solimp.push(DEFAULT_SOLIMP);
        model.jnt_name.push(Some("ball".to_string()));
        model.qpos_spring.extend_from_slice(&[1.0, 0.0, 0.0, 0.0]); // Ball: identity quat from qpos0

        // DOF definitions (3 for ball joint)
        for i in 0..3 {
            model.dof_body.push(1);
            model.dof_jnt.push(0);
            model
                .dof_parent
                .push(if i == 0 { None } else { Some(i - 1) });
            model.dof_armature.push(0.0);
            model.dof_damping.push(0.0);
            model.dof_frictionloss.push(0.0);
        }

        // Default qpos: identity quaternion [w, x, y, z] = [1, 0, 0, 0]
        model.qpos0 = DVector::from_vec(vec![1.0, 0.0, 0.0, 0.0]);

        // Physics options
        model.timestep = 1.0 / 240.0;
        model.gravity = Vector3::new(0.0, 0.0, -9.81);
        model.solver_iterations = 10;
        model.solver_tolerance = 1e-8;

        recompute(&mut model);
        model
    }

    /// Create a free-floating body (6-DOF).
    ///
    /// This creates a single body with a free joint, allowing full 3D
    /// translation and rotation. Useful for testing free-body dynamics.
    ///
    /// # Arguments
    /// * `mass` - Body mass (kg)
    /// * `inertia` - Principal moments of inertia [Ixx, Iyy, Izz]
    #[must_use]
    pub fn free_body(mass: f64, inertia: Vector3<f64>) -> Self {
        let mut model = Self::empty();

        // Dimensions
        model.nq = 7; // position (3) + quaternion (4)
        model.nv = 6; // linear velocity (3) + angular velocity (3)
        model.nbody = 2;
        model.njnt = 1;

        // Body 1: free body
        model.body_parent.push(0);
        model.body_rootid.push(1);
        model.body_jnt_adr.push(0);
        model.body_jnt_num.push(1);
        model.body_dof_adr.push(0);
        model.body_dof_num.push(6);
        model.body_geom_adr.push(0);
        model.body_geom_num.push(0);

        model.body_pos.push(Vector3::zeros());
        model.body_quat.push(UnitQuaternion::identity());
        model.body_ipos.push(Vector3::zeros());
        model.body_iquat.push(UnitQuaternion::identity());
        model.body_mass.push(mass);
        model.body_inertia.push(inertia);
        model.body_name.push(Some("free_body".to_string()));
        model.body_subtreemass.push(0.0); // Will be computed after model is built
        model.body_mocapid.push(None);

        // Free joint
        model.jnt_type.push(MjJointType::Free);
        model.jnt_body.push(1);
        model.jnt_qpos_adr.push(0);
        model.jnt_dof_adr.push(0);
        model.jnt_pos.push(Vector3::zeros());
        model.jnt_axis.push(Vector3::z());
        model.jnt_limited.push(false);
        model.jnt_range.push((-1e10, 1e10));
        model.jnt_stiffness.push(0.0);
        model.jnt_springref.push(0.0);
        model.jnt_damping.push(0.0);
        model.jnt_solref.push(DEFAULT_SOLREF);
        model.jnt_solimp.push(DEFAULT_SOLIMP);
        model.jnt_name.push(Some("free".to_string()));
        model
            .qpos_spring
            .extend_from_slice(&[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]); // Free: pos + identity quat

        // DOF definitions (6 for free joint)
        for i in 0..6 {
            model.dof_body.push(1);
            model.dof_jnt.push(0);
            model
                .dof_parent
                .push(if i == 0 { None } else { Some(i - 1) });
            model.dof_armature.push(0.0);
            model.dof_damping.push(0.0);
            model.dof_frictionloss.push(0.0);
        }

        // Default qpos: at origin with identity orientation
        model.qpos0 = DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);

        // Physics options
        model.timestep = 1.0 / 240.0;
        model.gravity = Vector3::new(0.0, 0.0, -9.81);
        model.solver_iterations = 10;
        model.solver_tolerance = 1e-8;

        recompute(&mut model);
        model
    }

    /// Add a ground plane geom at z = 0 (attached to the world body).
    ///
    /// The plane is infinite, collides with all other geoms, and uses
    /// contact parameters appropriate for mm-scale geometry.
    pub fn add_ground_plane(&mut self) {
        let geom_id = self.ngeom;
        self.ngeom += 1;

        let size = Vector3::new(40.0, 40.0, 0.1);
        self.geom_type.push(GeomType::Plane);
        self.geom_body.push(0);
        self.geom_pos.push(Vector3::zeros());
        self.geom_quat.push(UnitQuaternion::identity());
        self.geom_size.push(size);
        self.geom_friction.push(Vector3::new(1.0, 0.005, 0.0001));
        self.geom_condim.push(3);
        self.geom_contype.push(1);
        self.geom_conaffinity.push(1);
        self.geom_margin.push(0.0);
        self.geom_gap.push(0.0);
        self.geom_priority.push(0);
        self.geom_solmix.push(1.0);
        self.geom_solimp.push(DEFAULT_SOLIMP);
        self.geom_solref.push(DEFAULT_SOLREF);
        self.geom_fluid.push([0.0; 12]);
        self.geom_name.push(Some("ground".into()));
        self.geom_rbound.push(0.0);
        self.geom_aabb
            .push(local_aabb_from_primitive(GeomType::Plane, size));
        // At the world's origin, unturned: the world's frame.
        self.geom_sameframe.push(SameFrame::Body);
        self.geom_mesh.push(None);
        self.geom_hfield.push(None);
        self.geom_shape.push(None);
        self.geom_group.push(0);
        self.geom_rgba.push([0.5, 0.5, 0.5, 0.3]);
        self.geom_user.push(vec![]);
        self.geom_plugin.push(None);

        // Update world body's geom tracking
        self.body_geom_num[0] += 1;
        if self.body_geom_num[0] == 1 {
            self.body_geom_adr[0] = geom_id;
        }
        // The world's hierarchy, and margin, now hold the plane; the flexes'
        // hierarchies follow the bodies' and the meshes' nodes again.
        self.compute_body_bvhs();
        self.compute_flex_bvhs();
    }

    /// A model of one mesh geom on the world, the mesh's vertices `vert`
    /// (three numbers a vertex) and faces `face` (three vertices a face),
    /// derived: a fixture for the mesh's hierarchy and ray tests.
    #[cfg(test)]
    pub(crate) fn one_mesh(vert: &[f64], face: &[usize]) -> Self {
        let mut model = Self::empty();
        model.add_ground_plane();
        model.geom_type[0] = GeomType::Mesh;
        model.geom_mesh[0] = Some(0);
        let vertices = vert
            .chunks(3)
            .map(|v| nalgebra::Point3::new(v[0], v[1], v[2]))
            .collect();
        model
            .mesh_data
            .push(std::sync::Arc::new(crate::mesh::TriangleMeshData::new(
                vertices,
                face.to_vec(),
            )));
        model.mesh_name.push("mesh".into());
        model.nmesh = 1;
        model.compute_geom_bounding_radii();
        model.compute_body_bvhs();
        model.compute_flex_bvhs();
        model
    }
}

/// Size the per-element arrays a factory does not fill, to the defaults the
/// test-fixture builders push, then derive every field the primary fields
/// determine.
// A factory's model is fixed and valid, so a refusal is a bug in the factory.
#[allow(clippy::panic)]
fn recompute(model: &mut Model) {
    model.jnt_group.resize(model.njnt, 0);
    model.jnt_actgravcomp.resize(model.njnt, false);
    model.jnt_margin.resize(model.njnt, 0.0);
    model.jnt_user.resize(model.njnt, Vec::new());
    model.dof_solref.resize(model.nv, DEFAULT_SOLREF);
    model.dof_solimp.resize(model.nv, DEFAULT_SOLIMP);
    model.body_gravcomp.resize(model.nbody, 0.0);
    model.body_user.resize(model.nbody, Vec::new());
    model.body_plugin.resize(model.nbody, None);
    if let Err(e) = model.recompute_derived() {
        panic!("a factory model is refused: {e}");
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used)]
    use super::*;
    use crate::test_fixtures::checks::short_arrays;

    /// `short_arrays` checks every per-element `Vec` field `Model` declares
    /// for these kinds, read from `model.rs`, so a field added later is
    /// checked or this fails.
    #[test]
    fn short_arrays_checks_every_per_element_field() {
        const KINDS: [&str; 8] = [
            "jnt", "dof", "body", "geom", "site", "tendon", "actuator", "sensor",
        ];
        let mut declared: Vec<&str> = include_str!("model.rs")
            .lines()
            .filter_map(|line| line.trim().strip_prefix("pub ")?.split_once(": "))
            .filter(|(name, ty)| {
                ty.starts_with("Vec<") && name.split('_').next().is_some_and(|k| KINDS.contains(&k))
            })
            .map(|(name, _)| name)
            .collect();
        let (_, mut checked) = short_arrays(&Model::n_link_pendulum(1, 1.0, 0.1));
        declared.sort_unstable();
        checked.sort_unstable();
        assert_eq!(checked, declared);
    }

    /// The factories fill what they do not set with MuJoCo's defaults: no
    /// joint margin, no gravity compensation, the default solver parameters.
    #[test]
    fn factory_models_take_mujoco_defaults() {
        let models = vec![
            Model::n_link_pendulum(3, 1.0, 0.1),
            Model::multi_joint_body(),
            Model::spherical_pendulum(1.0, 0.1),
            Model::free_body(1.0, Vector3::new(0.1, 0.1, 0.1)),
        ];
        for m in &models {
            assert_eq!(m.jnt_margin, vec![0.0; m.njnt]);
            assert_eq!(m.jnt_actgravcomp, vec![false; m.njnt]);
            assert_eq!(m.body_gravcomp, vec![0.0; m.nbody]);
            assert_eq!(m.dof_solref, vec![DEFAULT_SOLREF; m.nv]);
            assert_eq!(m.dof_solimp, vec![DEFAULT_SOLIMP; m.nv]);
        }
    }

    /// A joint limit on a factory model assembles: the factories used to leave
    /// `jnt_margin` empty, and the first step panicked indexing it.
    #[test]
    fn a_limited_joint_steps_on_a_factory_model() {
        let mut model = Model::n_link_pendulum(2, 1.0, 0.1);
        model.jnt_limited[0] = true;
        model.jnt_range[0] = (-0.1, 0.1);
        let mut data = model.make_data();
        data.qpos[0] = 0.3;
        data.step(&model).expect("step");
        assert!(
            data.efc_type
                .contains(&crate::types::ConstraintType::LimitJoint)
        );
    }

    /// A factory or test-fixture model sizes every per-joint, per-dof,
    /// per-body, per-geom, per-site, per-tendon, per-actuator and per-sensor
    /// array to its element count, as the MJCF builder does. Not
    /// `collision_geoms`, which holds only the contact parameters the
    /// narrow-phase tests read and is never stepped.
    #[test]
    fn factory_models_size_every_per_element_array() {
        let mut with_ground = Model::n_link_pendulum(2, 1.0, 0.1);
        with_ground.add_ground_plane();
        let models = vec![
            ("n_link_pendulum", Model::n_link_pendulum(3, 1.0, 0.1)),
            ("double_pendulum", Model::double_pendulum(1.0, 0.1)),
            ("multi_joint_body", Model::multi_joint_body()),
            ("spherical_pendulum", Model::spherical_pendulum(1.0, 0.1)),
            (
                "free_body",
                Model::free_body(1.0, Vector3::new(0.1, 0.1, 0.1)),
            ),
            ("n_link_pendulum + ground plane", with_ground),
            ("hinge_chain", crate::test_fixtures::hinge_chain(2)),
            ("bistable_chain", crate::test_fixtures::bistable_chain(2)),
            ("cart_pole", crate::test_fixtures::cart_pole()),
            (
                "builder_test_minimal",
                crate::test_fixtures::builder_test_minimal(),
            ),
            (
                "free_body_diag",
                crate::test_fixtures::free_body_diag(1.0, Vector3::new(0.1, 0.2, 0.3)),
            ),
            (
                "hinge_chain_2dof_inertial",
                crate::test_fixtures::hinge_chain_2dof_inertial(),
            ),
            ("reaching_2dof", crate::test_fixtures::reaching_2dof()),
            ("reaching_6dof", crate::test_fixtures::reaching_6dof()),
            (
                "reaching_6dof_obstacle",
                crate::test_fixtures::reaching_6dof_obstacle(),
            ),
            ("pendulum_basic", crate::test_fixtures::pendulum_basic()),
            (
                "pendulum_with_angle_sensor",
                crate::test_fixtures::pendulum_with_angle_sensor(),
            ),
            ("pendulum_clamped", crate::test_fixtures::pendulum_clamped()),
            ("pendulum_mocap", crate::test_fixtures::pendulum_mocap()),
            (
                "pendulum_with_tip_site",
                crate::test_fixtures::pendulum_with_tip_site(),
            ),
            ("pendulum_bench", crate::test_fixtures::pendulum_bench()),
            ("sho_1d", crate::test_fixtures::sho_1d()),
            ("ratchet", crate::test_fixtures::ratchet()),
            (
                "stochastic_resonance",
                crate::test_fixtures::stochastic_resonance(),
            ),
            ("bistable_1dof", crate::test_fixtures::bistable_1dof()),
            ("single_slide", crate::test_fixtures::single_slide()),
            ("ising_pair", crate::test_fixtures::ising_pair()),
        ];
        for (name, model) in &models {
            let (short, _) = short_arrays(model);
            assert!(short.is_empty(), "{name}: {short:?}");
        }
    }

    #[test]
    fn multi_joint_body_is_well_formed_and_steps() {
        let model = Model::multi_joint_body();

        // Structural counts: one moving body carrying three scalar DOFs.
        assert_eq!(model.nbody, 2);
        assert_eq!(model.njnt, 3);
        assert_eq!(model.nq, 3);
        assert_eq!(model.nv, 3);
        assert_eq!(model.body_jnt_num[1], 3);
        assert_eq!(model.body_dof_num[1], 3);
        // hinge → slide → hinge.
        assert_eq!(model.jnt_type[0], MjJointType::Hinge);
        assert_eq!(model.jnt_type[1], MjJointType::Slide);
        assert_eq!(model.jnt_type[2], MjJointType::Hinge);

        // Forward dynamics runs and qM is symmetric positive-definite (a valid
        // multi-joint mass matrix), confirming the fixture finalizes correctly.
        let mut data = model.make_data();
        data.qpos.as_mut_slice().copy_from_slice(&[0.4, 0.15, -0.3]);
        data.forward(&model).expect("forward failed");
        for i in 0..model.nv {
            assert!(data.qM[(i, i)] > 0.0, "qM diagonal must be positive");
            for j in 0..model.nv {
                assert!(
                    (data.qM[(i, j)] - data.qM[(j, i)]).abs() < 1e-12,
                    "qM must be symmetric"
                );
            }
        }
    }
}
