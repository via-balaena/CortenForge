//! Factory methods for common mechanical systems.
//!
//! These constructors produce pre-configured [`Model`] instances for
//! canonical test systems (pendulums, free bodies, etc.). Used by
//! inline tests and by `sim-conformance-tests`.

use nalgebra::{DVector, UnitQuaternion, Vector3};
use std::f64::consts::PI;

use super::enums::MjJointType;
use super::model::Model;
use crate::mujoco_pipeline::{DEFAULT_SOLIMP, DEFAULT_SOLREF};

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

            // Body properties
            // Body frame is at the END of the link (where the mass is)
            model.body_pos.push(Vector3::new(0.0, 0.0, -link_length));
            model.body_quat.push(UnitQuaternion::identity());
            model.body_ipos.push(Vector3::zeros()); // COM at body origin
            model.body_iquat.push(UnitQuaternion::identity());
            model.body_mass.push(link_mass);
            // Point mass approximation (small moment of inertia)
            model.body_inertia.push(Vector3::new(0.001, 0.001, 0.001));
            model.body_name.push(Some(format!("link_{i}")));
            model.body_subtreemass.push(0.0); // Will be computed after model is built
            model.body_mocapid.push(None);

            // Joint definition (hinge at parent's frame, rotating around Y)
            model.jnt_type.push(MjJointType::Hinge);
            model.jnt_body.push(body_id);
            model.jnt_qpos_adr.push(i);
            model.jnt_dof_adr.push(i);
            model.jnt_pos.push(Vector3::zeros()); // Joint at body origin
            model.jnt_axis.push(Vector3::new(0.0, 1.0, 0.0)); // Rotate around Y
            model.jnt_limited.push(false);
            model.jnt_range.push((-PI, PI));
            model.jnt_stiffness.push(0.0);
            model.jnt_springref.push(0.0);
            model.jnt_damping.push(0.0);
            model.jnt_armature.push(0.0);
            model.jnt_solref.push(DEFAULT_SOLREF);
            model.jnt_solimp.push(DEFAULT_SOLIMP);
            model.jnt_name.push(Some(format!("hinge_{i}")));

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

        // Pre-compute kinematic data for O(n) CRBA/RNE
        model.compute_ancestors();

        // Pre-compute implicit integration parameters
        model.compute_implicit_params();

        // Pre-compute CSR sparsity metadata for sparse LDL factorization
        model.compute_qld_csr_metadata();

        model
    }

    /// Create a double pendulum (2-link serial chain).
    ///
    /// Convenience method equivalent to `Model::n_link_pendulum(2, ...)`.
    #[must_use]
    pub fn double_pendulum(link_length: f64, link_mass: f64) -> Self {
        Self::n_link_pendulum(2, link_length, link_mass)
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

        // Body frame is at the mass location (below joint by length)
        model.body_pos.push(Vector3::new(0.0, 0.0, -length));
        model.body_quat.push(UnitQuaternion::identity());
        model.body_ipos.push(Vector3::zeros());
        model.body_iquat.push(UnitQuaternion::identity());
        model.body_mass.push(mass);
        model.body_inertia.push(Vector3::new(0.001, 0.001, 0.001));
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
        model.jnt_armature.push(0.0);
        model.jnt_solref.push(DEFAULT_SOLREF);
        model.jnt_solimp.push(DEFAULT_SOLIMP);
        model.jnt_name.push(Some("ball".to_string()));

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

        // Pre-compute kinematic data for O(n) CRBA/RNE
        model.compute_ancestors();

        // Pre-compute implicit integration parameters
        model.compute_implicit_params();

        // Pre-compute CSR sparsity metadata for sparse LDL factorization
        model.compute_qld_csr_metadata();

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
        model.jnt_armature.push(0.0);
        model.jnt_solref.push(DEFAULT_SOLREF);
        model.jnt_solimp.push(DEFAULT_SOLIMP);
        model.jnt_name.push(Some("free".to_string()));

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

        // Pre-compute kinematic data for O(n) CRBA/RNE
        model.compute_ancestors();

        // Pre-compute implicit integration parameters
        model.compute_implicit_params();

        // Pre-compute CSR sparsity metadata for sparse LDL factorization
        model.compute_qld_csr_metadata();

        model
    }
}
