//! Recomputing a `Model`'s derived fields after an edit
//! ([`Model::recompute_derived`]), and two of the derivations: geom bounds
//! and fixed tendon lengths, moved here from sim-mjcf's builder so that every
//! `Model` producer can run them.

use cf_geometry::Bounded;
use nalgebra::Vector3;

use super::enums::{GeomType, ModelError, TendonType};
use super::model::Model;
use super::model_init::compute_dof_lengths;
use super::validation::MAX_VAL;

impl Model {
    /// Recompute every field derived from the model's primary fields, in the
    /// order building a model computes them, after [`Self::check_joint_layout`],
    /// [`Self::check_ranges`] and [`Self::check_actuators`]: the
    /// equivalent of MuJoCo's `mj_setConst` (3.5.0 `engine_setconst.c:1089-1101`)
    /// plus the muscle length-range simulation, which `mj_setConst` leaves out
    /// (`:1088`). The factories and the test fixtures' `finalize` run it; call
    /// it after editing a primary field.
    ///
    /// | After editing | Stale | Recomputed by |
    /// |---|---|---|
    /// | `jnt_stiffness`, `jnt_damping` (hinge, slide), `dof_damping` (ball, free), `qpos_spring` | `implicit_stiffness`, `implicit_damping`, `implicit_springref` | [`Self::compute_implicit_params`] |
    /// | the body and joint tree (`body_parent`, `body_jnt_*`, `body_dof_*`) | `body_ancestor_joints`, `body_ancestor_mask`, `body_weldid` | [`Self::compute_ancestors`] |
    /// | body, inertial, joint, geom and site frames, joint types | `body_sameframe`, `geom_sameframe`, `site_sameframe`, `body_simple`, `dof_simplenum` | [`Self::compute_simple_bodies`] |
    /// | `dof_parent`, `dof_simplenum` | the mass matrix's pattern and the implicit integrator's (`M_*`, `D_*`), so the lengths of `Data::qLD_data`, `Data::qH` and `Data::qLU`: make a new `Data` | [`Self::compute_dof_patterns`] |
    /// | masses, inertias, `dof_armature`, body poses, `qpos0`, joint axes | `dof_M0`, a simple dof's row of the mass matrix (at every state): first, as what follows builds the mass matrix at `qpos0` | [`Self::compute_dof_m0`] |
    /// | masses, inertias, body poses, `qpos0`, joint axes, flex equalities | `body_subtreemass`, `body_invweight0`, `dof_invweight0`, `tendon_invweight0`, `flex_edgeequality`, `flexedge_invweight0`, `stat_meaninertia` | [`Self::compute_invweight0`], [`Self::compute_stat_meaninertia`] |
    /// | gears, transmissions, joint and tendon ranges, masses | `actuator_lengthrange`, `actuator_acc0` | [`Self::compute_actuator_params`] |
    /// | spatial tendon geometry, `qpos0`, `qpos_spring` | spatial `tendon_length0`, `tendon_lengthspring` | [`Self::compute_spatial_tendon_length0`] |
    /// | fixed tendon wraps, `qpos0`, `qpos_spring` | fixed `tendon_length0`, `tendon_lengthspring` | [`Self::compute_fixed_tendon_lengths`] |
    /// | `geom_size`, `geom_type`, mesh, height field or SDF data | `geom_rbound`, `geom_aabb` | [`Self::compute_geom_bounding_radii`] |
    /// | geom boxes, frames, bodies, `geom_contype`, `geom_conaffinity`, `geom_margin`, body inertial frames, mesh data | `body_margin`, the bodies' and then the meshes' hierarchies (`body_bvhadr`, `body_bvhnum`, `mesh_bvhadr`, `mesh_bvhnum`, `bvh_*`), dropping the flexes' | [`Self::compute_body_bvhs`] |
    /// | flex elements | `flex_elemlayer`, `flex_evpair`, `flex_evpairadr`, `flex_evpairnum` | [`Self::compute_flex_topology`] |
    /// | flex elements, `flex_activelayers`, `flex_radius`, `qpos0` and the vertices' bodies, the bodies' and meshes' hierarchies | the flexes' hierarchies (`flex_bvhadr`, `flex_bvhnum`, `nbvhstatic`, `nbvhdynamic`, `bvh_*` past the meshes'), so `Data::bvh_aabb_dyn`'s length: make a new `Data` | [`Self::compute_flex_bvhs`] |
    /// | `dof_parent`, `body_dof_*`, `body_weldid`, actuators, tendons, flex vertex bodies | the tree tables, the tendon trees, the automatic sleep policies | [`Self::compute_kinematic_trees`] |
    /// | `qpos0` and what the position stage reads (body, joint and geom frames, `body_ipos`), `geom_rbound`, joint types | `dof_length` | [`compute_dof_lengths`] |
    /// | `body_gravcomp` | `ngravcomp`, the bodies with a positive value | this function |
    /// | `actuator_nsample`, `sensor_nsample`, `sensor_dim` | `actuator_historyadr`, `sensor_historyadr`, `nhistory`, so `Data::history`'s length: make a new `Data` | [`Self::compute_history_addresses`] |
    ///
    /// Some inputs are consumed on the first computation: a damping ratio in
    /// `actuator_biasprm[2]` (as MuJoCo's `set0` consumes it) and a
    /// `tendon_lengthspring` of `[-1, -1]` are replaced by the values derived
    /// from them; a later edit of the input is not derived again. Not
    /// recomputed here: the tree's own tables `body_rootid` and `dof_parent`,
    /// which an edit of the tree sets; and the flex edge Jacobian's pattern
    /// (`flexedge_J_rownnz`, `flexedge_J_rowadr`, `flexedge_J_colind`), the
    /// MJCF builder's, from each end body's own dofs, where MuJoCo's `set0`
    /// remakes it over each end's whole dof chain (`makeFlexSparse`,
    /// `engine_setconst.c:286`): the two differ once a vertex body has a dof
    /// above it.
    ///
    /// # Errors
    /// The [`ModelError`] `check_joint_layout`, `check_ranges` or
    /// `check_actuators` finds; the model is then unchanged.
    pub fn recompute_derived(&mut self) -> Result<(), ModelError> {
        self.check_joint_layout()?;
        self.check_ranges()?;
        self.check_actuators()?;
        // MuJoCo setFixed (engine_setconst.c:98-103).
        self.ngravcomp = self.body_gravcomp.iter().filter(|&&gc| gc > 0.0).count();
        // Before the derivations below, which make a `Data` sized by `nhistory`.
        self.compute_history_addresses();
        self.compute_ancestors();
        self.compute_simple_bodies();
        self.compute_dof_patterns();
        self.compute_implicit_params();
        self.compute_dof_m0();
        self.compute_spatial_tendon_length0();
        self.compute_actuator_params();
        self.compute_stat_meaninertia();
        self.compute_invweight0();
        self.compute_geom_bounding_radii();
        self.compute_body_bvhs();
        self.compute_flex_topology();
        self.compute_flex_bvhs();
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
    /// half-extents). For meshes and SDFs, bounds come from their data; for
    /// height fields and primitives, from their size.
    ///
    /// `geom_rbound` is the radius of a sphere about the geom's origin that
    /// holds the geom, as MuJoCo's `mjCGeom::GetRBound`
    /// (`user_objects.cc:3418-3454`): for a mesh or an SDF, the norm of each
    /// axis's larger bound in magnitude; for a height field, from its size,
    /// `sqrt(x² + y² + max(top², base²))`. The broadphase's bounding-sphere
    /// filters read it about the geom's origin. `geom_aabb` is MuJoCo's
    /// `mjModel.geom_aabb` (`mjCGeom::ComputeAABB`,
    /// `user_objects.cc:3573-3626`), which the bodies' hierarchies bound
    /// ([`Self::compute_body_bvhs`], to run after this) and the midphase
    /// tests.
    pub fn compute_geom_bounding_radii(&mut self) {
        for geom_id in 0..self.ngeom {
            let (rbound, aabb) = if let Some(mesh_id) = self.geom_mesh[geom_id] {
                // Mesh geom: bounds from actual vertex positions.
                let (aabb_min, aabb_max) = self.mesh_data[mesh_id].aabb();
                let center = (aabb_min.coords + aabb_max.coords) * 0.5;
                let half = (aabb_max.coords - aabb_min.coords) * 0.5;
                let rbound = origin_rbound(&aabb_min.coords, &aabb_max.coords);
                (
                    rbound,
                    [center.x, center.y, center.z, half.x, half.y, half.z],
                )
            } else if let Some(hfield_id) = self.geom_hfield[geom_id] {
                // Hfield geom: bounds from its size, its base to its top,
                // not from its data's heights.
                let s = self.hfield_size[hfield_id];
                let rbound = (s[0] * s[0] + s[1] * s[1] + (s[2] * s[2]).max(s[3] * s[3])).sqrt();
                (rbound, aabb_of([-s[0], -s[1], -s[3]], [s[0], s[1], s[2]]))
            } else if let Some(sdf_id) = self.geom_shape[geom_id] {
                // SDF geom: bounds from SDF grid data.
                let aabb = self.shape_data[sdf_id].sdf_grid().aabb();
                let center = (aabb.min.coords + aabb.max.coords) * 0.5;
                let half = (aabb.max.coords - aabb.min.coords) * 0.5;
                let rbound = origin_rbound(&aabb.min.coords, &aabb.max.coords);
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

/// The radius of the sphere about a geom's origin that holds a box from
/// `min` to `max` in the geom's frame: the norm of each axis's larger bound
/// in magnitude (MuJoCo's `mjCGeom::GetRBound` for a mesh or an SDF,
/// `user_objects.cc:3443-3449`).
fn origin_rbound(min: &Vector3<f64>, max: &Vector3<f64>) -> f64 {
    let h = min.abs().sup(&max.abs());
    (h.x * h.x + h.y * h.y + h.z * h.z).sqrt()
}

/// A primitive geom's box in its frame, `[cx, cy, cz, hx, hy, hz]`, as
/// MuJoCo's `mjCGeom::ComputeAABB` (`user_objects.cc:3573-3626`): about the
/// origin, with half-sizes from the geom's size, except a plane's, which is
/// the half-space below it, to [`MAX_VAL`] each way.
pub(super) fn local_aabb_from_primitive(geom_type: GeomType, size: Vector3<f64>) -> [f64; 6] {
    match geom_type {
        GeomType::Sphere => {
            let r = size.x;
            [0.0, 0.0, 0.0, r, r, r]
        }
        GeomType::Box | GeomType::Ellipsoid => [0.0, 0.0, 0.0, size.x, size.y, size.z],
        // Local z is the axis: a capsule's caps reach its radius past its
        // half-length, a cylinder ends at it.
        GeomType::Capsule => [0.0, 0.0, 0.0, size.x, size.x, size.x + size.y],
        GeomType::Cylinder => [0.0, 0.0, 0.0, size.x, size.x, size.y],
        GeomType::Plane => aabb_of([-MAX_VAL; 3], [MAX_VAL, MAX_VAL, 0.0]),
        // Mesh/Hfield/Sdf are handled by the caller using actual geometry data.
        // This arm is a fallback for programmatic geoms that bypass the builder.
        GeomType::Mesh | GeomType::Hfield | GeomType::Sdf => [0.0, 0.0, 0.0, 0.1, 0.1, 0.1],
    }
}

/// The box from `min` to `max` as `[cx, cy, cz, hx, hy, hz]`, its centre and
/// half-sizes computed as MuJoCo's `ComputeAABB` computes them
/// (`user_objects.cc:3619-3625`); `f64::midpoint` is `(max + min) / 2` for
/// bounds within `f64::MAX / 2`.
pub(super) fn aabb_of(min: [f64; 3], max: [f64; 3]) -> [f64; 6] {
    [
        f64::midpoint(max[0], min[0]),
        f64::midpoint(max[1], min[1]),
        f64::midpoint(max[2], min[2]),
        (max[0] - min[0]) / 2.0,
        (max[1] - min[1]) / 2.0,
        (max[2] - min[2]) / 2.0,
    ]
}

#[cfg(test)]
mod bounds_tests {
    use std::sync::Arc;

    use nalgebra::{Point3, Vector3};

    use crate::mesh::TriangleMeshData;
    use crate::sdf::{SdfGrid, ShapeConvex};
    use crate::types::{GeomType, Model};
    use crate::validation::max_or_nan;

    /// A model whose one geom, the ground plane's slot, is made `geom_type`.
    fn one_geom(geom_type: GeomType) -> Model {
        let mut model = Model::free_body(1.0, Vector3::new(0.01, 0.01, 0.01));
        model.add_ground_plane();
        model.geom_type[0] = geom_type;
        model
    }

    /// The distance from a geom's origin to the farthest corner of its box,
    /// `geom_aabb`.
    fn farthest_corner(aabb: [f64; 6]) -> f64 {
        (0..8_u32)
            .map(|corner| {
                let p: Vec<f64> = (0..3)
                    .map(|i| {
                        let sign = if corner >> i & 1 == 1 { 1.0 } else { -1.0 };
                        aabb[i] + sign * aabb[i + 3]
                    })
                    .collect();
                (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt()
            })
            .fold(0.0, max_or_nan)
    }

    /// The plane `add_ground_plane` appends after the model is derived has the
    /// box deriving gives a plane, MuJoCo's half-space below it.
    #[test]
    fn a_ground_plane_has_a_planes_box() {
        let mut model = Model::free_body(1.0, Vector3::new(0.01, 0.01, 0.01));
        model.add_ground_plane();
        let appended = model.geom_aabb[0];
        model.compute_geom_bounding_radii();
        assert!(appended.iter().all(|x| x.is_finite()), "{appended:?}");
        assert_eq!(
            appended.map(f64::to_bits),
            model.geom_aabb[0].map(f64::to_bits)
        );
        assert_eq!(
            appended[2].to_bits(),
            (-0.5e10_f64).to_bits(),
            "the centre of the half-space below"
        );
    }

    /// A box mesh 20 to 22 below its geom's origin: its bounding radius is
    /// MuJoCo's, about the origin, sqrt(1 + 1 + 22²), which reaches the far
    /// corners. The half-diagonal of the box, sqrt(3), is about the box's
    /// centre.
    #[test]
    fn an_off_centre_mesh_is_bounded_from_its_origin() {
        let mut model = one_geom(GeomType::Mesh);
        let vertices: Vec<Point3<f64>> = (0..8)
            .map(|k| {
                let x = if k & 1 == 1 { 1.0 } else { -1.0 };
                let y = if k & 2 == 2 { 1.0 } else { -1.0 };
                let z = if k & 4 == 4 { -20.0 } else { -22.0 };
                Point3::new(x, y, z)
            })
            .collect();
        let indices = vec![
            0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6, 0, 1, 4, 1, 5, 4, 2, 6, 3, 3, 6, 7, 0, 4, 2, 2, 4,
            6, 1, 3, 5, 3, 7, 5,
        ];
        model
            .mesh_data
            .push(Arc::new(TriangleMeshData::new(vertices, indices)));
        model.geom_mesh[0] = Some(0);
        model.compute_geom_bounding_radii();
        assert_eq!(
            model.geom_rbound[0].to_bits(),
            486.0_f64.sqrt().to_bits(),
            "rbound {}",
            model.geom_rbound[0]
        );
    }

    /// An SDF sphere 20 below its geom's origin: its bounding radius is the
    /// distance to the farthest corner of the grid's box, MuJoCo's rule for an
    /// SDF.
    #[test]
    fn an_off_centre_sdf_is_bounded_from_its_origin() {
        let mut model = one_geom(GeomType::Sdf);
        let sdf = SdfGrid::sphere(Point3::new(0.0, 0.0, -20.0), 1.0, 8, 0.5);
        model.shape_data = vec![Arc::new(ShapeConvex::new(Arc::new(sdf)))];
        model.geom_shape[0] = Some(0);
        model.compute_geom_bounding_radii();
        let far = farthest_corner(model.geom_aabb[0]);
        assert!(far > 21.5, "the grid's box reaches {far}");
        assert!(
            (model.geom_rbound[0] - far).abs() <= 1e-12 * far,
            "rbound {}, farthest corner {far}",
            model.geom_rbound[0]
        );
    }
}
