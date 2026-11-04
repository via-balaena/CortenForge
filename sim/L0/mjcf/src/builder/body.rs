//! Body tree traversal.
//!
//! Processes MJCF `<body>` elements into `Model` body arrays, recursively
//! traversing the kinematic tree. Handles worldbody geoms/sites, mocap
//! validation, inertia computation dispatch, and child body recursion.

use nalgebra::{UnitQuaternion, Vector3};

use super::ModelBuilder;
use super::geom::{compute_geom_mass, geom_volume};
use super::mass::{
    MJ_MINVAL, check_replaced_full_inertia, compute_inertia_from_geoms,
    extract_inertial_properties, geom_hfield_size, geom_mesh_props,
};
use super::orientation::{MJ_EPS, resolve_orientation};
use crate::error::{ElementKind, Location, MjcfError, Result};
use crate::keywords;
use crate::types::{InertiaFromGeom, MjcfBody, MjcfGeom};
use sim_core::GeomType;

/// What MuJoCo 3.5.0's `mjCBody::Compile` refuses in one body, found as the
/// body is built and raised in MuJoCo's order by
/// [`ModelBuilder::check_bodies`].
#[derive(Debug, Clone, Copy, Default)]
pub struct BodyFaults {
    /// A `fullinertia` beside a diagonal inertia other than 0
    /// (`user_objects.cc:2408-2409`).
    pub full_and_diagonal: bool,
    /// The first of its geoms whose mass or density MuJoCo gives a negative
    /// value where the geom infers its body's inertia
    /// ([`ModelBuilder::negative_mass`]).
    pub negative_geom: Option<usize>,
    /// Its mass or a principal moment is negative after `boundmass` and
    /// `boundinertia` ([`ModelBuilder::apply_mass_pipeline`],
    /// `user_objects.cc:2457-2459`).
    pub negative: bool,
    /// Its principal moments, after `boundinertia`, break A + B >= C, and
    /// `balanceinertia` is off ([`ModelBuilder::apply_mass_pipeline`]).
    pub triangle: bool,
}

impl ModelBuilder {
    /// Process geoms and sites directly attached to worldbody (body 0).
    ///
    /// In MJCF, the worldbody can have geoms (like ground planes) and sites
    /// directly attached to it. These are static geometries at world coordinates.
    ///
    /// Each geom and site reaches it with its class applied and its `fromto`
    /// resolved (`resolve::apply_defaults`).
    pub(crate) fn process_worldbody_geoms_and_sites(&mut self, worldbody: &MjcfBody) -> Result<()> {
        // Track geom start address for body 0
        let geom_adr = self.geom_type.len();

        // Process worldbody geoms
        for geom in &worldbody.geoms {
            self.process_geom(geom, 0)?;
        }

        // Process worldbody sites
        for site in &worldbody.sites {
            self.process_site(site, 0)?;
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
    ) -> Result<usize> {
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
    ) -> Result<usize> {
        let body_id = self.body_parent.len();

        // Validate mocap body constraints (before any state mutations)
        if body.mocap {
            let at = || Location::nth(ElementKind::Body, Some(&body.name), body_id);
            if parent_id != 0 {
                return Err(MjcfError::invalid_model(
                    "a mocap body must be a direct child of worldbody",
                )
                .at(at()));
            }
            if !body.joints.is_empty() {
                return Err(MjcfError::invalid_model(format!(
                    "a mocap body must not have joints (has {})",
                    body.joints.len()
                ))
                .at(at()));
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
        // Its alternative where it has one, else its quaternion made unit
        // (`resolve_orientation`).
        let body_pos = body.pos;
        let body_quat = resolve_orientation(
            body.quat,
            body.euler,
            body.axisangle,
            body.xyaxes,
            body.zaxis,
            &self.compiler,
        )
        .map_err(|e| {
            e.on("body")
                .at(Location::nth(ElementKind::Body, Some(&body.name), body_id))
        })?;

        // Compute world frame position for this body (used for free joint qpos0)
        let world_pos = parent_world_pos + parent_world_quat * body_pos;
        let world_quat = parent_world_quat * body_quat;

        // Store world positions for child body processing
        self.body_world_pos.push(world_pos);
        self.body_world_quat.push(world_quat);

        let (mass, inertia, ipos, iquat) =
            self.resolve_body_inertia(body, &body.geoms).map_err(|e| {
                e.with_location_if_unknown(Location::nth(
                    ElementKind::Body,
                    Some(&body.name),
                    body_id,
                ))
            })?;

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
        let sleep_policy = body
            .sleep
            .as_deref()
            .map(|s| {
                keywords::body_sleep(s).ok_or_else(|| {
                    MjcfError::invalid_keyword("body", "sleep", s).at(Location::nth(
                        ElementKind::Body,
                        Some(&body.name),
                        body_id,
                    ))
                })
            })
            .transpose()?;
        self.body_sleep_policy.push(sleep_policy);
        self.body_gravcomp.push(body.gravcomp.unwrap_or(0.0));
        self.body_user_raw.push(body.user.clone());

        // Process joints for this body, tracking the last DOF for kinematic tree linkage
        // MuJoCo semantics: first DOF of first joint links to parent body's last DOF,
        // subsequent DOFs form a chain within and across joints
        let mut body_nv = 0;
        let mut current_last_dof = parent_last_dof;

        for joint in &body.joints {
            let jnt_id =
                self.process_joint(joint, body_id, current_last_dof, world_pos, world_quat)?;
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

        // Process geoms for this body
        for geom in &body.geoms {
            self.process_geom(geom, body_id)?;
        }
        self.body_geom_num.push(body.geoms.len());
        // A geom infers its body's inertia where the body writes no
        // `<inertial>` or `inertiafromgeom` is true (`user_objects.cc:2429-2432`;
        // MuJoCo also asks the geom's group to be in `inertiagrouprange`, which
        // this crate does not read, ledger L375).
        let infers =
            body.inertial.is_none() || self.compiler.inertiafromgeom == InertiaFromGeom::True;
        let negative = body
            .geoms
            .iter()
            .position(|geom| infers && self.negative_mass(geom));
        let full_and_diagonal = body.inertial.as_ref().is_some_and(|inertial| {
            inertial.fullinertia.is_some()
                && inertial.diaginertia.is_some_and(|d| d != Vector3::zeros())
        });
        self.body_faults.push(BodyFaults {
            full_and_diagonal,
            negative_geom: negative.map(|k| geom_adr + k),
            negative: false,
            triangle: false,
        });
        // Under `inertiafromgeom="true"` MuJoCo replaces the `<inertial>`
        // only where it selects a geom, one whose mass is above mjEPS
        // (`InertiaFromGeom`, `user_objects.cc:2163-2170`; the group range
        // is not read, ledger L375); where it selects none, it keeps the
        // `<inertial>` and raises a negative value to its bound.
        if self.compiler.inertiafromgeom == InertiaFromGeom::True
            && self.kept_negative_inertial.is_none()
            && !body.geoms.iter().any(|geom| self.geom_mass(geom) > MJ_EPS)
        {
            self.kept_negative_inertial = body
                .inertial
                .as_ref()
                .and_then(crate::validation::negative_inertial_error)
                .map(|error| error.at(self.body_location(body_id)));
        }

        // Process sites for this body
        for site in &body.sites {
            self.process_site(site, body_id)?;
        }

        // Recursively process children, passing this body's last DOF and world frame
        for child in &body.children {
            self.process_body_with_world_frame(
                child,
                body_id,
                current_last_dof,
                world_pos,
                world_quat,
            )?;
        }

        Ok(body_id)
    }

    /// Whether MuJoCo 3.5.0's compiler gives `geom` a negative mass or
    /// density (`mjCGeom::Compile`, `user_objects.cc:3780-3807`): a written
    /// mass of 0 neither; any other written mass its own sign where the
    /// geom's volume is above mjEPS, and the written density's where it is
    /// not; with no mass written, the density's.
    fn negative_mass(&self, geom: &MjcfGeom) -> bool {
        let density = geom.density.unwrap_or(1000.0);
        match geom.mass {
            Some(0.0) => false,
            Some(mass) if self.volume(geom) > MJ_EPS => mass < 0.0,
            Some(_) | None => density < 0.0,
        }
    }

    /// A geom's volume as the mass rule reads it ([`geom_volume`]), from
    /// its mesh's or height field's asset.
    fn volume(&self, geom: &MjcfGeom) -> f64 {
        let mesh = geom_mesh_props(
            geom,
            &self.mesh_name_to_id,
            &self.mesh_data,
            &self.mesh_inertia_modes,
        );
        let hfield = geom_hfield_size(geom, &self.hfield_name_to_id, &self.hfield_size);
        geom_volume(geom, mesh.as_ref(), hfield)
    }

    /// A geom's mass as MuJoCo 3.5.0's mass rule gives it
    /// ([`compute_geom_mass`]), from its mesh's or height field's asset.
    fn geom_mass(&self, geom: &MjcfGeom) -> f64 {
        let mesh = geom_mesh_props(
            geom,
            &self.mesh_name_to_id,
            &self.mesh_data,
            &self.mesh_inertia_modes,
        );
        let hfield = geom_hfield_size(geom, &self.hfield_name_to_id, &self.hfield_size);
        compute_geom_mass(geom, mesh.as_ref(), hfield)
    }

    /// MuJoCo 3.5.0's checks of each body, body by body in its order from the
    /// world, as `mjCBody::Compile` makes them: a `fullinertia` beside a
    /// diagonal inertia (`user_objects.cc:2408-2409`); geom by geom, its
    /// checks ([`Self::check_geom`]); a mass or moment negative after the
    /// bounds (`:2457-2459`), then the moments' triangle
    /// ([`Self::apply_mass_pipeline`], `:2461-2469`); then the body's joints
    /// ([`Self::check_body_joints`]). A geom `fusestatic` moved into an
    /// earlier body is checked at its own body's turn, before the first body
    /// MuJoCo compiles after that one ([`super::compiler::CompileTurns`]):
    /// MuJoCo compiles every body before it fuses. The caller runs it with
    /// the checks RL-6a added, after every check this crate makes before
    /// building the model.
    pub(crate) fn check_bodies(&self) -> Result<()> {
        let nbody = self.body_parent.len();
        let mut moving = vec![false; nbody];
        for body in 1..nbody {
            moving[body] = self.body_jnt_num[body] > 0 || moving[self.body_parent[body]];
        }
        let turns = self.compile_turns.as_ref();
        // A body the fused tree does not hold (a flexcomp's vertex body)
        // after every one it does.
        let body_turn = |body: usize| {
            turns.map_or(body, |t| {
                t.body.get(body).copied().unwrap_or(t.name.len() + body)
            })
        };
        let geom_turn = |geom: usize| {
            turns
                .and_then(|t| t.geom.get(geom).copied())
                .unwrap_or_else(|| body_turn(self.geom_body[geom]))
        };
        let mut moved: Vec<usize> = (0..self.geom_body.len())
            .filter(|&geom| geom_turn(geom) != body_turn(self.geom_body[geom]))
            .collect();
        // Stable: one body's geoms as written.
        moved.sort_by_key(|&geom| geom_turn(geom));
        let mut moved = moved.into_iter().peekable();
        let written_in = |geom: usize| {
            let name = turns.map_or("", |t| t.name[geom_turn(geom)].as_str());
            Location::nth(ElementKind::Geom, self.geom_name[geom].as_deref(), geom).in_body(name)
        };
        for body in 0..nbody {
            while let Some(geom) = moved.next_if(|&geom| geom_turn(geom) < body_turn(body)) {
                self.check_geom(geom, moving[self.geom_body[geom]], || written_in(geom))?;
            }
            let faults = self.body_faults[body];
            if faults.full_and_diagonal {
                return Err(MjcfError::invalid_value(
                    "inertial",
                    "fullinertia",
                    "fullinertia and diagonal inertia cannot both be specified",
                )
                .at(self.body_location(body)));
            }
            let start = self.body_geom_adr[body];
            for geom in start..start + self.body_geom_num[body] {
                if geom_turn(geom) != body_turn(body) {
                    continue;
                }
                self.check_geom(geom, moving[body], || {
                    Location::nth(ElementKind::Geom, self.geom_name[geom].as_deref(), geom)
                        .within_nth(ElementKind::Body, self.body_name[body].as_deref(), body)
                })?;
            }
            if faults.negative {
                return Err(
                    MjcfError::invalid_model("mass and inertia cannot be negative")
                        .at(self.body_location(body)),
                );
            }
            if faults.triangle {
                return Err(MjcfError::invalid_model(
                    "inertia must satisfy A + B >= C; use 'balanceinertia' to fix",
                )
                .at(self.body_location(body)));
            }
            self.check_body_joints(body)?;
        }
        for geom in moved {
            self.check_geom(geom, moving[self.geom_body[geom]], || written_in(geom))?;
        }
        Ok(())
    }

    /// MuJoCo 3.5.0's checks of a geom in `mjCGeom::Compile`'s order: a
    /// condim other than 1, 3, 4 or 6 (`user_objects.cc:3651-3654`), a plane
    /// only in a static body, one with no joint on it or a body above it
    /// (`:3666-3668`; fusion moves a geom only into a body that moves as its
    /// own does), then a negative mass or density ([`Self::negative_mass`],
    /// `:3805-3807`). The world's geoms can fail only their condim.
    fn check_geom(&self, geom: usize, moving: bool, at: impl Fn() -> Location) -> Result<()> {
        if !matches!(self.geom_condim[geom], 1 | 3 | 4 | 6) {
            return Err(MjcfError::invalid_model("invalid condim in geom").at(at()));
        }
        if moving && self.geom_type[geom] == GeomType::Plane {
            return Err(MjcfError::invalid_model("plane only allowed in static bodies").at(at()));
        }
        if self.body_faults[self.geom_body[geom]].negative_geom == Some(geom) {
            return Err(
                MjcfError::invalid_model("mass, inertia or density are negative in geom").at(at()),
            );
        }
        Ok(())
    }

    /// MuJoCo 3.5.0's check of every body that moves, one with a joint
    /// (`user_model.cc:4993-4998`): its mass and each principal moment at
    /// least mjMINVAL, or a body below it with no joint, reached through
    /// bodies with no joint, that has them (`CheckBodyMassInertia`,
    /// `:5310-5331`). A flexcomp's vertex bodies are checked as MuJoCo
    /// checks them, as bodies. The first such body is refused; the caller
    /// runs it after the sensors' checks and before the free joints', as
    /// MuJoCo orders them, and before `settotalmass`.
    pub(crate) fn check_moving_body_mass(&self) -> Result<()> {
        let nbody = self.body_parent.len();
        let mut carries: Vec<bool> = (0..nbody)
            .map(|b| {
                self.body_mass[b] >= MJ_MINVAL
                    && self.body_inertia[b]
                        .iter()
                        .all(|&moment| moment >= MJ_MINVAL)
            })
            .collect();
        // A child's id is above its parent's, so each body is final before
        // it reaches its parent.
        for body in (1..nbody).rev() {
            if self.body_jnt_num[body] == 0 && carries[body] {
                carries[self.body_parent[body]] = true;
            }
        }
        match (1..nbody).find(|&b| self.body_jnt_num[b] > 0 && !carries[b]) {
            Some(body) => Err(MjcfError::invalid_model(
                "mass and inertia of moving bodies must be larger than mjMINVAL",
            )
            .at(self.body_location(body))),
            None => Ok(()),
        }
    }

    /// Resolve inertial properties with full MuJoCo semantics.
    ///
    /// Dispatches based on `compiler.inertiafromgeom`:
    /// - `True`  — always compute from geoms (overrides explicit `<inertial>`,
    ///   whose `fullinertia` MuJoCo still refuses where it would refuse it)
    /// - `Auto`  — compute from geoms only when no explicit `<inertial>`
    /// - `False` — use explicit `<inertial>` or zero
    fn resolve_body_inertia(
        &self,
        body: &MjcfBody,
        resolved_geoms: &[MjcfGeom],
    ) -> Result<(f64, Vector3<f64>, Vector3<f64>, UnitQuaternion<f64>)> {
        match self.compiler.inertiafromgeom {
            InertiaFromGeom::True => {
                check_replaced_full_inertia(body.inertial.as_ref())?;
                compute_inertia_from_geoms(
                    resolved_geoms,
                    &self.mesh_name_to_id,
                    &self.mesh_data,
                    &self.mesh_inertia_modes,
                    &self.hfield_name_to_id,
                    &self.hfield_size,
                )
            }
            InertiaFromGeom::Auto => {
                if let Some(ref inertial) = body.inertial {
                    extract_inertial_properties(inertial)
                } else {
                    compute_inertia_from_geoms(
                        resolved_geoms,
                        &self.mesh_name_to_id,
                        &self.mesh_data,
                        &self.mesh_inertia_modes,
                        &self.hfield_name_to_id,
                        &self.hfield_size,
                    )
                }
            }
            InertiaFromGeom::False => {
                if let Some(ref inertial) = body.inertial {
                    extract_inertial_properties(inertial)
                } else {
                    Ok((
                        0.0,
                        Vector3::zeros(),
                        Vector3::zeros(),
                        UnitQuaternion::identity(),
                    ))
                }
            }
        }
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use crate::builder::load_model;
    use crate::builder::orientation::euler_seq_to_quat;
    use nalgebra::{UnitQuaternion, Vector3};
    use sim_core::GeomType;

    /// Test site parsing and model population.
    #[test]
    fn test_sites() {
        let model = load_model(
            r#"
            <mujoco model="site_test">
                <worldbody>
                    <body name="arm" pos="0 0 1">
                        <joint name="j1" type="hinge" axis="0 1 0"/>
                        <inertial pos="0 0 0" mass="1.0" diaginertia="0.1 0.1 0.1"/>
                        <geom type="capsule" size="0.05 0.5"/>
                        <site name="end_effector" pos="0 0 -1" size="0.02"/>
                        <site name="sensor_mount" pos="0.1 0 0" size="0.01 0.01 0.02"/>
                    </body>
                </worldbody>
            </mujoco>
        "#,
        )
        .expect("Should parse");

        assert_eq!(model.nsite, 2);
        assert_eq!(model.site_body.len(), 2);

        // Check first site (end_effector)
        assert_eq!(model.site_body[0], 1); // Attached to arm (body 1)
        assert!((model.site_pos[0].z - (-1.0)).abs() < 1e-10);
        assert_eq!(model.site_name[0], Some("end_effector".to_string()));

        // Check second site (sensor_mount)
        assert_eq!(model.site_body[1], 1);
        assert!((model.site_pos[1].x - 0.1).abs() < 1e-10);
    }

    // =========================================================================
    // Site orientation tests (item #21)
    // =========================================================================

    /// AC1: Site euler orientation with default angle="degree".
    #[test]
    fn test_site_euler_orientation_degrees() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" euler="90 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let expected = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 90.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site euler 90deg X deg: got {got:?}, expected {expected:?}"
        );
    }

    /// AC2: Site euler orientation with explicit angle="radian".
    #[test]
    fn test_site_euler_orientation_radians() {
        let pi_2 = std::f64::consts::FRAC_PI_2;
        let model = load_model(&format!(
            r#"
            <mujoco>
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" euler="{pi_2} 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        ))
        .expect("should load");

        let expected = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), pi_2);
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site euler pi/2 X rad: got {got:?}, expected {expected:?}"
        );
    }

    /// AC3: Site euler with non-default eulerseq="ZYX".
    #[test]
    fn test_site_euler_orientation_zyx_eulerseq() {
        let model = load_model(
            r#"
            <mujoco>
                <compiler angle="radian" eulerseq="ZYX"/>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" euler="0.3 0.2 0.1"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let expected = euler_seq_to_quat(Vector3::new(0.3, 0.2, 0.1), "ZYX");
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site euler ZYX: got {got:?}, expected {expected:?}"
        );
    }

    /// AC4: Site axisangle orientation with default angle="degree".
    #[test]
    fn test_site_axisangle_orientation_degrees() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" axisangle="0 0 1 90"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 90.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site axisangle 90deg Z deg: got {got:?}, expected {expected:?}"
        );
    }

    /// AC5: Site axisangle orientation with angle="radian".
    #[test]
    fn test_site_axisangle_orientation_radians() {
        let pi_2 = std::f64::consts::FRAC_PI_2;
        let model = load_model(&format!(
            r#"
            <mujoco>
                <compiler angle="radian"/>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" axisangle="0 1 0 {pi_2}"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        ))
        .expect("should load");

        let expected = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), pi_2);
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site axisangle pi/2 Y rad: got {got:?}, expected {expected:?}"
        );
    }

    /// AC6: Site xyaxes orientation with orthogonal inputs (90deg Z rotation).
    #[test]
    fn test_site_xyaxes_orientation_orthogonal() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" xyaxes="0 1 0 -1 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // x=(0,1,0), y=(-1,0,0), z=cross(x,y)=(0,0,1) -> 90deg about Z
        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 90.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site xyaxes orthogonal 90deg Z: got {got:?}, expected {expected:?}"
        );
    }

    /// AC7: Site xyaxes with non-orthogonal inputs (Gram-Schmidt -> 45deg Z rotation).
    #[test]
    fn test_site_xyaxes_orientation_gram_schmidt() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" xyaxes="1 1 0 0 1 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Gram-Schmidt: x=norm(1,1,0)=(1/sqrt2, 1/sqrt2, 0), y orthogonalized -> 45deg about Z
        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 45.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-6,
            "site xyaxes Gram-Schmidt 45deg Z: got {got:?}, expected {expected:?}"
        );
    }

    /// AC8: Site zaxis general direction (Z->X = 90deg about Y).
    #[test]
    fn test_site_zaxis_orientation_general() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" zaxis="1 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Minimal rotation from (0,0,1) to (1,0,0): 90deg about Y
        let expected =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f64::consts::FRAC_PI_2);
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site zaxis Z->X (90deg Y): got {got:?}, expected {expected:?}"
        );
    }

    /// AC9: Site zaxis parallel to default Z -> identity.
    #[test]
    fn test_site_zaxis_orientation_parallel_identity() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" zaxis="0 0 1"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        let expected = UnitQuaternion::identity();
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site zaxis (0,0,1) identity: got {got:?}, expected {expected:?}"
        );
    }

    /// AC10: Site zaxis anti-parallel (0,0,-1) -> 180deg about X.
    #[test]
    fn test_site_zaxis_orientation_antiparallel() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" zaxis="0 0 -1"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Anti-parallel: fallback axis = X, angle = atan2(0, -1) = PI -> 180deg about X
        let expected = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::PI);
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site zaxis anti-parallel 180deg X: got {got:?}, expected {expected:?}"
        );
    }

    /// AC11: Site quat orientation (regression for pre-#19 behavior).
    #[test]
    fn test_site_quat_orientation_regression() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" quat="0.7071068 0 0.7071068 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Direct quaternion (wxyz): 90deg about Y
        let expected =
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f64::consts::FRAC_PI_2);
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-6,
            "site quat regression 90deg Y: got {got:?}, expected {expected:?}"
        );
    }

    /// AC12: Site orientation with default class (defaults provide type/size,
    /// element provides orientation — no interference).
    #[test]
    fn test_site_orientation_with_default_class() {
        let model = load_model(
            r#"
            <mujoco>
                <default>
                    <default class="sensor_site">
                        <site type="cylinder" size="0.02 0.01"/>
                    </default>
                </default>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" class="sensor_site" euler="0 0 90"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Type from default class
        assert_eq!(
            model.site_type[0],
            GeomType::Cylinder,
            "site type should come from default class"
        );

        // Orientation from element (not interfered by defaults)
        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 90.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site euler 90deg Z with default class: got {got:?}, expected {expected:?}"
        );
    }

    /// AC13: a site with both an euler and a quat is refused, as MuJoCo
    /// 3.5.0's `ReadAlternative` refuses it ("multiple orientation specifiers
    /// are not allowed").
    #[test]
    fn test_site_with_euler_and_quat_is_refused() {
        let err = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" euler="90 0 0" quat="1 0 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .unwrap_err();
        assert!(
            matches!(&err, crate::error::MjcfError::ConflictingAttributes { element, attributes, .. }
                if element == "site" && attributes == &["quat", "euler"]),
            "{err}"
        );
    }

    /// AC14: Site orientation composed with frame rotation.
    #[test]
    fn test_site_orientation_in_frame() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <frame euler="0 0 90">
                            <site name="s" euler="90 0 0"/>
                        </frame>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Frame: 90deg Z, Site: 90deg X. Composed: frame_q * site_q
        let frame_q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 90.0_f64.to_radians());
        let site_q = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 90.0_f64.to_radians());
        let expected = frame_q * site_q;
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-6,
            "site in frame composed: got {got:?}, expected {expected:?}"
        );
    }

    /// AC15: Site axisangle with non-unit axis (normalization).
    #[test]
    fn test_site_axisangle_non_unit_axis() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" axisangle="0 0 3 90"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // Axis (0,0,3) normalizes to (0,0,1) -> same as axisangle="0 0 1 90"
        let expected = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 90.0_f64.to_radians());
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site axisangle non-unit axis: got {got:?}, expected {expected:?}"
        );
    }

    /// AC16: Site zaxis with non-unit direction (normalization, parallel to Z).
    #[test]
    fn test_site_zaxis_non_unit_direction() {
        let model = load_model(
            r#"
            <mujoco>
                <worldbody>
                    <body name="b">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                        <site name="s" zaxis="0 0 5"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("should load");

        // (0,0,5) normalizes to (0,0,1) -> parallel to default Z -> identity
        let expected = UnitQuaternion::identity();
        let got = model.site_quat[0];
        assert!(
            (got.into_inner() - expected.into_inner()).norm() < 1e-10,
            "site zaxis non-unit parallel: got {got:?}, expected {expected:?}"
        );
    }
}
