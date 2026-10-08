//! Recomputing a `Model`'s derived fields after an edit
//! ([`Model::recompute_derived`]), and two of the derivations: geom bounds
//! and fixed tendon lengths, moved here from sim-mjcf's builder so that every
//! `Model` producer can run them.

use cf_geometry::Bounded;
use nalgebra::Vector3;

use super::enums::{GeomType, ModelError, TendonType};
use super::model::Model;
use super::model_init::compute_dof_lengths;

impl Model {
    /// Recompute every field derived from the model's primary fields, in the
    /// order building a model computes them, after [`Self::check_joint_layout`]
    /// and [`Self::check_ranges`]: the equivalent of MuJoCo's `mj_setConst` (3.5.0
    /// `engine_setconst.c:1089-1101`) plus the muscle length-range simulation,
    /// which `mj_setConst` leaves out (`:1088`). The factories and the test
    /// fixtures' `finalize` run it; call it after editing a primary field.
    ///
    /// | After editing | Stale | Recomputed by |
    /// |---|---|---|
    /// | `jnt_stiffness`, `jnt_damping` (hinge, slide), `dof_damping` (ball, free), `qpos_spring` | `implicit_stiffness`, `implicit_damping`, `implicit_springref` | [`Self::compute_implicit_params`] |
    /// | the body and joint tree (`body_parent`, `body_jnt_*`, `body_dof_*`) | `body_ancestor_joints`, `body_ancestor_mask`, `body_weldid` | [`Self::compute_ancestors`] |
    /// | `dof_parent` | the sparse factor's layout (`qLD_*`), so `Data::qLD_data`'s length: make a new `Data` | [`Self::compute_qld_csr_metadata`] |
    /// | masses, inertias, body poses, `qpos0`, joint axes | `body_subtreemass`, `body_invweight0`, `dof_invweight0`, `tendon_invweight0`, `stat_meaninertia` | [`Self::compute_invweight0`], [`Self::compute_stat_meaninertia`] |
    /// | gears, transmissions, joint and tendon ranges, masses | `actuator_lengthrange`, `actuator_acc0` | [`Self::compute_actuator_params`] |
    /// | spatial tendon geometry, `qpos0` | spatial `tendon_length0` | [`Self::compute_spatial_tendon_length0`] |
    /// | fixed tendon wraps, `qpos0`, `qpos_spring` | fixed `tendon_length0`, `tendon_lengthspring` | [`Self::compute_fixed_tendon_lengths`] |
    /// | `geom_size`, `geom_type`, mesh, height field or SDF data | `geom_rbound`, `geom_aabb` | [`Self::compute_geom_bounding_radii`] |
    /// | `dof_parent`, `body_dof_*`, `body_weldid`, actuators, tendons, flex vertex bodies | the tree tables, the tendon trees, the automatic sleep policies | [`Self::compute_kinematic_trees`] |
    /// | `qpos0` and what the position stage reads (body, joint and geom frames, `body_ipos`), `geom_rbound`, joint types | `dof_length` | [`compute_dof_lengths`] |
    /// | `body_gravcomp` | `ngravcomp`, the bodies with a positive value | this function |
    /// | `actuator_nsample`, `sensor_nsample`, `sensor_dim` | `actuator_historyadr`, `sensor_historyadr`, `nhistory`, so `Data::history`'s length: make a new `Data` | [`Self::compute_history_addresses`] |
    ///
    /// Some inputs are consumed on the first computation: a damping ratio in
    /// `actuator_biasprm[2]` (as MuJoCo's `set0` consumes it), a muscle's
    /// `actuator_gainprm[2]` (MuJoCo resolves that one at each call instead)
    /// and a `tendon_lengthspring` of `[-1, -1]` are replaced by the values
    /// derived from them; a later edit of the input is not derived again. Not
    /// recomputed here: the tree's own tables `body_rootid` and `dof_parent`,
    /// which an edit of the tree sets.
    ///
    /// # Errors
    /// The [`ModelError`] `check_joint_layout` or `check_ranges` finds; the
    /// model is then unchanged.
    pub fn recompute_derived(&mut self) -> Result<(), ModelError> {
        self.check_joint_layout()?;
        self.check_ranges()?;
        // MuJoCo setFixed (engine_setconst.c:98-103).
        self.ngravcomp = self.body_gravcomp.iter().filter(|&&gc| gc > 0.0).count();
        // Before the derivations below, which make a `Data` sized by `nhistory`.
        self.compute_history_addresses();
        self.compute_ancestors();
        self.compute_implicit_params();
        self.compute_qld_csr_metadata();
        self.compute_spatial_tendon_length0();
        self.compute_actuator_params();
        self.compute_stat_meaninertia();
        self.compute_invweight0();
        self.compute_geom_bounding_radii();
        self.compute_fixed_tendon_lengths();
        self.compute_kinematic_trees();
        compute_dof_lengths(self);
        Ok(())
    }

    /// Lay out the history buffers in `Data::history`: actuators first, then
    /// sensors, each in index order, `2 + 2 n` entries for an actuator and
    /// `2 + n + n dim` for a sensor with `n = nsample > 0`; the address is -1
    /// without a buffer. Sets `actuator_historyadr`, `sensor_historyadr` and
    /// `nhistory`, as MuJoCo's compiler does (`user_model.cc:3760-3770`,
    /// `:3806-3819`, sizes `:2252-2264`).
    pub fn compute_history_addresses(&mut self) {
        let mut offset = 0;
        self.actuator_historyadr = self
            .actuator_nsample
            .iter()
            .map(|&n| {
                if n <= 0 {
                    return -1;
                }
                let adr = offset;
                offset += 2 + 2 * n;
                adr
            })
            .collect();
        self.sensor_historyadr = self
            .sensor_nsample
            .iter()
            .zip(&self.sensor_dim)
            .map(|(&n, &dim)| {
                if n <= 0 {
                    return -1;
                }
                let adr = offset;
                // MuJoCo stores sensor_dim and nhistory as ints
                #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
                let dim = dim as i32;
                offset += 2 + n + n * dim;
                adr
            })
            .collect();
        // a running sum of positive sizes
        #[allow(clippy::cast_sign_loss)]
        {
            self.nhistory = offset as usize;
        }
    }

    /// Pre-compute bounding volumes for all geoms (collision broad-phase).
    ///
    /// Populates both `geom_rbound` (bounding sphere radius) and `geom_aabb`
    /// (local-frame AABB as `[cx, cy, cz, hx, hy, hz]` — center offset +
    /// half-extents). For meshes/hfield/sdf, bounds come from actual geometry
    /// data. For primitives, bounds come from `geom_size`.
    ///
    /// `geom_aabb` is transformed to world-space at runtime by `mj_collision`,
    /// replacing the per-type dispatch in `aabb_from_geom`. This eliminates the
    /// `MESH_DEFAULT_EXTENT` fallback and matches MuJoCo's `mjModel.geom_aabb`.
    pub fn compute_geom_bounding_radii(&mut self) {
        for geom_id in 0..self.ngeom {
            let (rbound, aabb) = if let Some(mesh_id) = self.geom_mesh[geom_id] {
                // Mesh geom: bounds from actual vertex positions.
                let (aabb_min, aabb_max) = self.mesh_data[mesh_id].aabb();
                let center = (aabb_min.coords + aabb_max.coords) * 0.5;
                let half = (aabb_max.coords - aabb_min.coords) * 0.5;
                let rbound = half.norm();
                (
                    rbound,
                    [center.x, center.y, center.z, half.x, half.y, half.z],
                )
            } else if let Some(hfield_id) = self.geom_hfield[geom_id] {
                // Hfield geom: bounds from heightfield data.
                let aabb = self.hfield_data[hfield_id].aabb();
                let center = (aabb.min.coords + aabb.max.coords) * 0.5;
                let half = (aabb.max.coords - aabb.min.coords) * 0.5;
                let rbound = half.norm();
                (
                    rbound,
                    [center.x, center.y, center.z, half.x, half.y, half.z],
                )
            } else if let Some(sdf_id) = self.geom_shape[geom_id] {
                // SDF geom: bounds from SDF grid data.
                let aabb = self.shape_data[sdf_id].sdf_grid().aabb();
                let center = (aabb.min.coords + aabb.max.coords) * 0.5;
                let half = (aabb.max.coords - aabb.min.coords) * 0.5;
                let rbound = half.norm();
                (
                    rbound,
                    [center.x, center.y, center.z, half.x, half.y, half.z],
                )
            } else {
                // Primitive geom: local AABB from size parameters.
                let rbound = self.geom_type[geom_id].bounding_radius(self.geom_size[geom_id]);
                let aabb =
                    local_aabb_from_primitive(self.geom_type[geom_id], self.geom_size[geom_id]);
                (rbound, aabb)
            };
            self.geom_rbound[geom_id] = rbound;
            self.geom_aabb[geom_id] = aabb;
        }
    }

    /// Pre-compute fixed tendon `length0` and resolve `lengthspring` sentinels from `qpos0`/`qpos_spring`.
    ///
    /// For fixed tendons: `length0 = Σ coef_w * qpos0[jnt_qposadr_w]`.
    /// MuJoCo ref: `setSpring()` in `engine_setconst.c` uses `qpos_spring` for sentinel resolution.
    pub fn compute_fixed_tendon_lengths(&mut self) {
        for t in 0..self.ntendon {
            if self.tendon_type[t] == TendonType::Fixed {
                let adr = self.tendon_adr[t];
                let num = self.tendon_num[t];
                // Compute tendon_length0 at qpos0 configuration
                let mut length0 = 0.0;
                for w in adr..(adr + num) {
                    let dof_adr = self.wrap_objid[w];
                    let coef = self.wrap_prm[w];
                    if dof_adr < self.nv {
                        let jnt_id = self.dof_jnt[dof_adr];
                        let qpos_adr = self.jnt_qpos_adr[jnt_id];
                        if qpos_adr < self.qpos0.len() {
                            length0 += coef * self.qpos0[qpos_adr];
                        }
                    }
                }
                self.tendon_length0[t] = length0;

                // Resolve sentinel [-1, -1] at qpos_spring configuration (not qpos0).
                #[allow(clippy::float_cmp)]
                if self.tendon_lengthspring[t] == [-1.0, -1.0] {
                    let mut spring_length = 0.0;
                    for w in adr..(adr + num) {
                        let dof_adr = self.wrap_objid[w];
                        let coef = self.wrap_prm[w];
                        if dof_adr < self.nv {
                            let jnt_id = self.dof_jnt[dof_adr];
                            let qpos_adr = self.jnt_qpos_adr[jnt_id];
                            if qpos_adr < self.qpos_spring.len() {
                                spring_length += coef * self.qpos_spring[qpos_adr];
                            }
                        }
                    }
                    self.tendon_lengthspring[t] = [spring_length, spring_length];
                }
            }
        }
    }
}

/// Compute local-frame AABB `[cx, cy, cz, hx, hy, hz]` for a primitive geom.
///
/// Center offset is `(0,0,0)` for all primitives (symmetric about their origin).
/// Half-extents come from the MuJoCo size convention for each type.
fn local_aabb_from_primitive(geom_type: GeomType, size: Vector3<f64>) -> [f64; 6] {
    match geom_type {
        GeomType::Sphere => {
            let r = size.x;
            [0.0, 0.0, 0.0, r, r, r]
        }
        GeomType::Box | GeomType::Ellipsoid => [0.0, 0.0, 0.0, size.x, size.y, size.z],
        GeomType::Capsule | GeomType::Cylinder => {
            // Local Z is the axis. Half-extents: radius in XY, radius+half_length in Z.
            let r = size.x;
            let h = size.y;
            [0.0, 0.0, 0.0, r, r, r + h]
        }
        GeomType::Plane => {
            // Infinite extent. The PLANE_EXTENT constant is applied at runtime
            // in aabb_from_geom_aabb since it depends on world-frame orientation.
            const INF: f64 = 1e6;
            [0.0, 0.0, 0.0, INF, INF, INF]
        }
        // Mesh/Hfield/Sdf are handled by the caller using actual geometry data.
        // This arm is a fallback for programmatic geoms that bypass the builder.
        GeomType::Mesh | GeomType::Hfield | GeomType::Sdf => [0.0, 0.0, 0.0, 0.1, 0.1, 0.1],
    }
}
