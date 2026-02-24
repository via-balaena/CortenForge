//! Body tree traversal.
//!
//! Processes MJCF `<body>` elements into `Model` body arrays, recursively
//! traversing the kinematic tree. Handles worldbody geoms/sites, mocap
//! validation, inertia computation dispatch, and child body recursion.

use nalgebra::{UnitQuaternion, Vector3};
use sim_core::SleepPolicy;
use tracing::warn;

use super::mass::{compute_inertia_from_geoms, extract_inertial_properties};
use super::orientation::resolve_orientation;
use super::{ModelBuilder, ModelConversionError};
use crate::types::{InertiaFromGeom, MjcfBody, MjcfGeom};

impl ModelBuilder {
    /// Process geoms and sites directly attached to worldbody (body 0).
    ///
    /// In MJCF, the worldbody can have geoms (like ground planes) and sites
    /// directly attached to it. These are static geometries at world coordinates.
    ///
    /// No childclass is applied here — MuJoCo's worldbody accepts no attributes
    /// (including childclass), so worldbody elements use only their own explicit
    /// class or the unnamed top-level default.
    pub(crate) fn process_worldbody_geoms_and_sites(
        &mut self,
        worldbody: &MjcfBody,
    ) -> std::result::Result<(), ModelConversionError> {
        // Track geom start address for body 0
        let geom_adr = self.geom_type.len();

        // Process worldbody geoms
        for geom in &worldbody.geoms {
            let geom = self.resolver.apply_to_geom(geom);
            self.process_geom(&geom, 0)?;
        }

        // Process worldbody sites
        for site in &worldbody.sites {
            let site = self.resolver.apply_to_site(site);
            self.process_site(&site, 0)?;
        }

        // Update body 0's geom range
        let num_geoms = self.geom_type.len() - geom_adr;
        self.body_geom_adr[0] = geom_adr;
        self.body_geom_num[0] = num_geoms;

        Ok(())
    }

    /// Process a body and its descendants.
    ///
    /// This is the public-facing entry point that initializes world frame tracking.
    ///
    /// # Arguments
    /// * `body` - The MJCF body to process
    /// * `parent_id` - Index of the parent body in the Model
    /// * `parent_last_dof` - Index of the last DOF in the parent body's kinematic chain
    ///   (None for bodies attached to world)
    ///
    /// # Returns
    /// The body index in the Model
    pub(crate) fn process_body(
        &mut self,
        body: &MjcfBody,
        parent_id: usize,
        parent_last_dof: Option<usize>,
        inherited_childclass: Option<&str>,
    ) -> std::result::Result<usize, ModelConversionError> {
        // Start with parent's world frame (world body is at origin)
        let (parent_world_pos, parent_world_quat) = if parent_id == 0 {
            (Vector3::zeros(), UnitQuaternion::identity())
        } else {
            // For non-world parents, we need to look up their world positions
            // Since we process in topological order, parent is already processed
            // We stored parent's world position when we added them
            (
                self.body_world_pos[parent_id - 1],
                self.body_world_quat[parent_id - 1],
            )
        };
        self.process_body_with_world_frame(
            body,
            parent_id,
            parent_last_dof,
            parent_world_pos,
            parent_world_quat,
            inherited_childclass,
        )
    }

    /// Internal body processing with world frame tracking.
    fn process_body_with_world_frame(
        &mut self,
        body: &MjcfBody,
        parent_id: usize,
        parent_last_dof: Option<usize>,
        parent_world_pos: Vector3<f64>,
        parent_world_quat: UnitQuaternion<f64>,
        inherited_childclass: Option<&str>,
    ) -> std::result::Result<usize, ModelConversionError> {
        let body_id = self.body_parent.len();

        // Validate mocap body constraints (before any state mutations)
        if body.mocap {
            if parent_id != 0 {
                return Err(ModelConversionError {
                    message: format!(
                        "mocap body '{}' must be a direct child of worldbody",
                        body.name
                    ),
                });
            }
            if !body.joints.is_empty() {
                return Err(ModelConversionError {
                    message: format!(
                        "mocap body '{}' must not have joints (has {})",
                        body.name,
                        body.joints.len()
                    ),
                });
            }
        }

        // Store name mapping
        if !body.name.is_empty() {
            self.body_name_to_id.insert(body.name.clone(), body_id);
        }

        // Determine root: if parent is world, this body is its own root
        let root_id = if parent_id == 0 {
            body_id
        } else {
            self.body_rootid[parent_id]
        };

        // Body position/orientation relative to parent.
        // Priority: euler > axisangle > xyaxes > zaxis > quat (MuJoCo convention).
        let body_pos = body.pos;
        let body_quat = resolve_orientation(
            body.quat,
            body.euler,
            body.axisangle,
            None,
            None,
            &self.compiler,
        );

        // Compute world frame position for this body (used for free joint qpos0)
        let world_pos = parent_world_pos + parent_world_quat * body_pos;
        let world_quat = parent_world_quat * body_quat;

        // Store world positions for child body processing
        self.body_world_pos.push(world_pos);
        self.body_world_quat.push(world_quat);

        // Determine effective childclass for this body's children
        let effective_childclass = body.childclass.as_deref().or(inherited_childclass);

        // Resolve geom defaults once for both inertia computation and geom processing.
        // Apply childclass: if a geom has no explicit class, set it from effective_childclass.
        let resolved_geoms: Vec<MjcfGeom> = body
            .geoms
            .iter()
            .map(|g| {
                let mut g = g.clone();
                if g.class.is_none() {
                    g.class = effective_childclass.map(|s| s.to_string());
                }
                self.resolver.apply_to_geom(&g)
            })
            .collect();

        // Process inertial properties with full MuJoCo semantics.
        // Gated by compiler.inertiafromgeom:
        //   True  — always compute from geoms (overrides explicit <inertial>)
        //   Auto  — compute from geoms only when no explicit <inertial>
        //   False — use explicit <inertial> or zero
        let (mass, inertia, ipos, iquat) = match self.compiler.inertiafromgeom {
            InertiaFromGeom::True => {
                compute_inertia_from_geoms(&resolved_geoms, &self.mesh_name_to_id, &self.mesh_data)
            }
            InertiaFromGeom::Auto => {
                if let Some(ref inertial) = body.inertial {
                    extract_inertial_properties(inertial)
                } else {
                    compute_inertia_from_geoms(
                        &resolved_geoms,
                        &self.mesh_name_to_id,
                        &self.mesh_data,
                    )
                }
            }
            InertiaFromGeom::False => {
                if let Some(ref inertial) = body.inertial {
                    extract_inertial_properties(inertial)
                } else {
                    (
                        0.0,
                        Vector3::zeros(),
                        Vector3::zeros(),
                        UnitQuaternion::identity(),
                    )
                }
            }
        };

        // Track joint/DOF addresses for this body
        let jnt_adr = self.jnt_type.len();
        let dof_adr = self.nv;
        let geom_adr = self.geom_type.len();

        // Add body arrays
        self.body_parent.push(parent_id);
        self.body_rootid.push(root_id);
        self.body_jnt_adr.push(jnt_adr);
        self.body_dof_adr.push(dof_adr);
        self.body_geom_adr.push(geom_adr);
        self.body_pos.push(body_pos);
        self.body_quat.push(body_quat);
        self.body_ipos.push(ipos);
        self.body_iquat.push(iquat);
        self.body_mass.push(mass);
        self.body_inertia.push(inertia);
        if body.mocap {
            self.body_mocapid.push(Some(self.nmocap));
            self.nmocap += 1;
        } else {
            self.body_mocapid.push(None);
        }
        self.body_name.push(if body.name.is_empty() {
            None
        } else {
            Some(body.name.clone())
        });
        // Parse body-level sleep policy
        let sleep_policy = body.sleep.as_ref().and_then(|s| match s.as_str() {
            "auto" => Some(SleepPolicy::Auto),
            "allowed" => Some(SleepPolicy::Allowed),
            "never" => Some(SleepPolicy::Never),
            "init" => Some(SleepPolicy::Init),
            _ => {
                warn!(
                    "body '{}': unknown sleep policy '{}', ignoring",
                    body.name, s
                );
                None
            }
        });
        self.body_sleep_policy.push(sleep_policy);
        self.body_gravcomp.push(body.gravcomp.unwrap_or(0.0));

        // Process joints for this body, tracking the last DOF for kinematic tree linkage
        // MuJoCo semantics: first DOF of first joint links to parent body's last DOF,
        // subsequent DOFs form a chain within and across joints
        let mut body_nv = 0;
        let mut current_last_dof = parent_last_dof;

        for joint in &body.joints {
            let mut joint = joint.clone();
            if joint.class.is_none() {
                joint.class = effective_childclass.map(|s| s.to_string());
            }
            let joint = self.resolver.apply_to_joint(&joint);
            let jnt_id =
                self.process_joint(&joint, body_id, current_last_dof, world_pos, world_quat)?;
            let jnt_nv = self.jnt_type[jnt_id].nv();
            body_nv += jnt_nv;

            // Update current_last_dof to the last DOF of this joint
            if jnt_nv > 0 {
                current_last_dof = Some(self.nv - 1);
            }
        }

        // Update body joint/DOF counts
        self.body_jnt_num.push(body.joints.len());
        self.body_dof_num.push(body_nv);

        // Process geoms for this body (using pre-resolved defaults)
        for geom in &resolved_geoms {
            self.process_geom(geom, body_id)?;
        }
        self.body_geom_num.push(body.geoms.len());

        // Process sites for this body
        for site in &body.sites {
            let mut site = site.clone();
            if site.class.is_none() {
                site.class = effective_childclass.map(|s| s.to_string());
            }
            let site = self.resolver.apply_to_site(&site);
            self.process_site(&site, body_id)?;
        }

        // Recursively process children, passing this body's last DOF and world frame
        for child in &body.children {
            self.process_body_with_world_frame(
                child,
                body_id,
                current_last_dof,
                world_pos,
                world_quat,
                effective_childclass,
            )?;
        }

        Ok(body_id)
    }
}
