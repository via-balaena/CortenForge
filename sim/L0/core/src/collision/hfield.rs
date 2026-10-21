//! Height field collision: MuJoCo's `mjc_ConvexHField`
//! ([`mjc_convex_hfield`]), the convex geom against the triangular prisms of
//! the grid under it, one contact per prism it overlaps through MuJoCo's
//! native convex collision, up to `mjMAXCONPAIR`.

use super::mj_ccd::{CcdObj, mjc_convex_hfield};
use super::mj_primitive::Mat9;
use crate::types::{Contact, GeomType, Model};
use nalgebra::{Matrix3, Vector3};

/// A height field against a convex geom, either first, through
/// [`mjc_convex_hfield`]: each contact listed with the height field first
/// (MuJoCo's type order), its normal from the field toward the geom. None
/// for a geom `mjc_ccd` does not take (a plane, a height field, a signed
/// distance field, a mesh without vertices).
// MuJoCo's pair context, as the other collision functions take it.
#[allow(clippy::too_many_arguments)]
pub fn collide_hfield_multi(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    margin: f64,
) -> Vec<Contact> {
    let (hf_geom, conv_geom, hf_pos, hf_mat, conv_pos, conv_mat) =
        if model.geom_type[geom1] == GeomType::Hfield {
            (geom1, geom2, pos1, mat1, pos2, mat2)
        } else {
            (geom2, geom1, pos2, mat2, pos1, mat1)
        };
    let Some(hfield_id) = model.geom_hfield[hf_geom] else {
        return vec![];
    };
    let Some(mut obj) = CcdObj::of_geom(model, conv_geom, &conv_pos, &conv_mat, 0.0) else {
        return vec![];
    };
    let mat: Mat9 = std::array::from_fn(|i| hf_mat[(i / 3, i % 3)]);
    mjc_convex_hfield(
        &model.hfield_data[hfield_id],
        &model.hfield_size[hfield_id],
        &[hf_pos.x, hf_pos.y, hf_pos.z],
        &mat,
        &mut obj,
        model.geom_rbound[conv_geom],
        margin,
        (model.ccd_iterations, model.ccd_tolerance),
    )
    .iter()
    .map(|c| c.contact(model, hf_geom, conv_geom, margin))
    .collect()
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::cast_precision_loss)]
mod tests {
    use super::*;
    use crate::heightfield::HeightFieldData;
    use crate::types::Model;
    use nalgebra::{Point3, UnitQuaternion};
    use std::sync::Arc;

    /// Build a minimal Model with one hfield geom (index 0) and one convex geom (index 1).
    ///
    /// `hf_heights`: flat-row-major heights (already pre-scaled).
    /// `hf_width`/`hf_depth`: grid dimensions.
    /// `hf_size`: MuJoCo [x_half, y_half, z_top_scale, z_bottom].
    /// `conv_type`: GeomType of the convex geom.
    /// `conv_size`: geom_size of the convex geom.
    /// `conv_rbound`: bounding sphere radius for the convex geom.
    fn make_hfield_model(
        hf_heights: Vec<f64>,
        hf_width: usize,
        hf_depth: usize,
        hf_size: [f64; 4],
        conv_type: GeomType,
        conv_size: Vector3<f64>,
        conv_rbound: f64,
    ) -> Model {
        let cell_size = if hf_width > 1 {
            2.0 * hf_size[0] / (hf_width - 1) as f64
        } else {
            1.0
        };
        let hfield = HeightFieldData::new(hf_heights, hf_width, hf_depth, cell_size);

        let mut model = Model::empty();
        model.ngeom = 2;
        model.nbody = 2;
        model.body_parent.push(0); // body 1 parent = world
        model.body_rootid.push(1);
        model.body_jnt_adr.push(0);
        model.body_jnt_num.push(0);
        model.body_dof_adr.push(0);
        model.body_dof_num.push(0);
        model.body_geom_adr.push(1);
        model.body_geom_num.push(1);
        model.body_pos.push(Vector3::zeros());
        model.body_quat.push(UnitQuaternion::identity());
        model.body_ipos.push(Vector3::zeros());
        model.body_iquat.push(UnitQuaternion::identity());
        model.body_mass.push(1.0);
        model.body_inertia.push(Vector3::new(1.0, 1.0, 1.0));
        model.body_name.push(None);
        model.body_subtreemass.push(1.0);
        model.body_mocapid.push(None);
        model.body_gravcomp.push(0.0);
        model.body_invweight0.push([1.0; 2]);
        model.body_treeid.push(0);

        model.geom_type = vec![GeomType::Hfield, conv_type];
        model.geom_body = vec![0, 1];
        model.geom_pos = vec![Vector3::zeros(); 2];
        model.geom_quat = vec![UnitQuaternion::identity(); 2];
        model.geom_size = vec![Vector3::zeros(), conv_size];
        model.geom_name = vec![None; 2];
        model.geom_rbound = vec![f64::INFINITY, conv_rbound];
        model.geom_mesh = vec![None; 2];
        model.geom_hfield = vec![Some(0), None];
        model.geom_shape = vec![None; 2];
        model.geom_contype = vec![1; 2];
        model.geom_conaffinity = vec![1; 2];
        model.geom_friction = vec![Vector3::new(1.0, 0.005, 0.0001); 2];
        model.geom_condim = vec![3; 2];
        model.geom_solref = vec![[0.02, 1.0]; 2];
        model.geom_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]; 2];
        model.geom_priority = vec![0; 2];
        model.geom_solmix = vec![1.0; 2];
        model.geom_margin = vec![0.0; 2];
        model.geom_gap = vec![0.0; 2];
        model.geom_group = vec![0; 2];
        model.geom_rgba = vec![[1.0; 4]; 2];
        model.geom_fluid = vec![[0.0; 12]; 2];

        model.hfield_data = vec![Arc::new(hfield)];
        model.hfield_size = vec![hf_size];
        model.mesh_data = vec![];
        model
    }

    /// Build a flat heightfield at the given height.
    /// Grid spans [-x_half..+x_half] x [-y_half..+y_half] with `n×n` vertices.
    fn flat_hfield_model(
        n: usize,
        x_half: f64,
        y_half: f64,
        height: f64,
        conv_type: GeomType,
        conv_size: Vector3<f64>,
        conv_rbound: f64,
    ) -> Model {
        let heights = vec![height; n * n];
        make_hfield_model(
            heights,
            n,
            n,
            [x_half, y_half, 1.0, 1.0], // z_top_scale=1.0, z_bottom=1.0
            conv_type,
            conv_size,
            conv_rbound,
        )
    }

    // ========================================================================
    // T1: Hfield-sphere prism contact → AC1
    // ========================================================================
    #[test]
    fn t1_hfield_sphere_prism_contact() {
        // 5×5 flat hfield at z=0.5, sphere radius=0.3 at (0,0,0.7)
        // Sphere bottom at z=0.4, below surface at z=0.5 → penetrating.
        let model = flat_hfield_model(
            5,
            2.0,
            2.0,
            0.5,
            GeomType::Sphere,
            Vector3::new(0.3, 0.0, 0.0),
            0.3,
        );
        let pos1 = Vector3::zeros(); // hfield at origin
        let mat1 = Matrix3::identity();
        let pos2 = Vector3::new(0.0, 0.0, 0.7);
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);

        assert!(!contacts.is_empty(), "Expected at least 1 contact");
        let c = &contacts[0];
        // Depth should be approximately 0.1 (sphere bottom at 0.4, surface at 0.5).
        // Tolerance ±0.05 per spec.
        assert!(
            c.depth > 0.05 && c.depth < 0.15,
            "Depth {} should be ≈0.1 (±0.05)",
            c.depth
        );
        // Normal should be approximately +Z (flat terrain)
        assert!(c.normal.z > 0.5, "Normal Z {} should be upward", c.normal.z);
    }

    // ========================================================================
    // T2: Hfield-mesh contact (new pair) → AC2
    // ========================================================================
    #[test]
    fn t2_hfield_mesh_with_hull() {
        use crate::mesh::TriangleMeshData;

        // Build a unit cube mesh centered at origin with a convex hull.
        let s = 0.5;
        let verts = vec![
            Point3::new(-s, -s, -s),
            Point3::new(s, -s, -s),
            Point3::new(s, s, -s),
            Point3::new(-s, s, -s),
            Point3::new(-s, -s, s),
            Point3::new(s, -s, s),
            Point3::new(s, s, s),
            Point3::new(-s, s, s),
        ];
        #[rustfmt::skip]
        let indices = vec![
            0,1,2, 0,2,3, // -Z face
            4,6,5, 4,7,6, // +Z face
            0,4,5, 0,5,1, // -Y face
            2,6,7, 2,7,3, // +Y face
            0,3,7, 0,7,4, // -X face
            1,5,6, 1,6,2, // +X face
        ];
        let mut mesh = TriangleMeshData::new(verts, indices);
        mesh.compute_convex_hull(None);
        assert!(mesh.convex_hull().is_some(), "Hull should be computed");

        // 5×5 flat hfield at z=0.0
        let mut model = flat_hfield_model(
            5,
            2.0,
            2.0,
            0.0,
            GeomType::Mesh,
            Vector3::new(0.5, 0.5, 0.5),
            0.87, // rbound ≈ sqrt(0.5² + 0.5² + 0.5²)
        );
        model.mesh_data = vec![Arc::new(mesh)];
        model.geom_mesh = vec![None, Some(0)]; // geom 1 = mesh index 0

        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        // Cube center at z=0.2 → bottom face at z=-0.3, deeply penetrating terrain at z=0.0
        let pos2 = Vector3::new(0.0, 0.0, 0.2);
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            !contacts.is_empty(),
            "Mesh with convex hull penetrating terrain should produce contacts"
        );
        // Verify at least one contact has positive depth (`mjc_ConvexHField`
        // lists a prism only where it and the mesh overlap).
        let max_depth = contacts.iter().map(|c| c.depth).fold(0.0_f64, f64::max);
        assert!(
            max_depth > 0.0,
            "At least one contact should have positive depth, max was {max_depth}",
        );
    }

    // ========================================================================
    // T3: Hfield-cylinder exact (no capsule approximation) → AC3
    // ========================================================================
    #[test]
    fn t3_hfield_cylinder_exact() {
        // Cylinder (radius=0.5, half_length=1.0) on its side, penetrating flat terrain at z=0.
        let model = flat_hfield_model(
            5,
            2.0,
            2.0,
            0.0,
            GeomType::Cylinder,
            Vector3::new(0.5, 1.0, 0.0), // MuJoCo: size[0]=radius, size[1]=half_length
            1.2,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        // Cylinder at z=0.3 with radius=0.5 → bottom at z=-0.2 (below terrain at z=0)
        let pos2 = Vector3::new(0.0, 0.0, 0.3);
        let mat2 = Matrix3::identity(); // Z-axis up = standing

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            !contacts.is_empty(),
            "Cylinder penetrating terrain should produce contacts"
        );
    }

    // ========================================================================
    // T4: Hfield-ellipsoid exact (no sphere approximation) → AC4
    // ========================================================================
    #[test]
    fn t4_hfield_ellipsoid_exact() {
        // Ellipsoid (radii 0.5, 0.3, 0.2) at z=0.15 → bottom at z=-0.05
        let model = flat_hfield_model(
            5,
            2.0,
            2.0,
            0.0,
            GeomType::Ellipsoid,
            Vector3::new(0.5, 0.3, 0.2),
            0.5,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        let pos2 = Vector3::new(0.0, 0.0, 0.15);
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            !contacts.is_empty(),
            "Ellipsoid penetrating terrain should produce contacts"
        );
    }

    // ========================================================================
    // T5: Multi-contact generation → AC5
    // ========================================================================
    #[test]
    fn t5_multi_contact_generation() {
        // 10×10 heightfield with sinusoidal terrain. Large box spanning multiple cells.
        let n = 10;
        let x_half = 5.0;
        let y_half = 5.0;
        let mut heights = Vec::with_capacity(n * n);
        for y in 0..n {
            for x in 0..n {
                let fx = x as f64 / (n - 1) as f64 * std::f64::consts::TAU;
                let fy = y as f64 / (n - 1) as f64 * std::f64::consts::TAU;
                heights.push(0.5 * (fx.sin() + fy.sin()) + 1.0);
            }
        }
        let model = make_hfield_model(
            heights,
            n,
            n,
            [x_half, y_half, 1.0, 2.0],
            GeomType::Box,
            Vector3::new(2.0, 2.0, 0.5), // Large box: 4×4×1
            3.0,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        // Box center at z=0.5 → bottom at z=0.0, terrain peaks ~2.0 → overlapping
        let pos2 = Vector3::new(0.0, 0.0, 0.5);
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            contacts.len() >= 4,
            "Large box on sinusoidal terrain should produce ≥4 contacts, got {}",
            contacts.len()
        );
    }

    // ========================================================================
    // T6: mjMAXCONPAIR = 50 limit → AC6
    // ========================================================================
    #[test]
    fn t6_max_contacts_limit() {
        // 50×50 heightfield, very large box spanning entire grid, deeply penetrating.
        let n = 50;
        let x_half = 25.0;
        let y_half = 25.0;
        let heights = vec![1.0; n * n]; // Flat at z=1.0
        let model = make_hfield_model(
            heights,
            n,
            n,
            [x_half, y_half, 1.0, 2.0],
            GeomType::Box,
            Vector3::new(24.0, 24.0, 0.5), // Huge box spanning almost entire grid
            35.0,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        // Box at z=0.0 → bottom at z=-0.5, terrain at z=1.0 → deep penetration
        let pos2 = Vector3::new(0.0, 0.0, 0.0);
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            contacts.len() <= 50,
            "Should be capped at 50 contacts, got {}",
            contacts.len()
        );
        assert!(
            contacts.len() == 50,
            "With so many cells, should hit the 50-contact limit, got {}",
            contacts.len()
        );
    }

    // ========================================================================
    // T7: Geom outside hfield bounds → AC7
    // ========================================================================
    #[test]
    fn t7_outside_bounds_no_contact() {
        let model = flat_hfield_model(
            5,
            2.0,
            2.0,
            0.0,
            GeomType::Sphere,
            Vector3::new(0.3, 0.0, 0.0),
            0.3,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        let pos2 = Vector3::new(100.0, 100.0, 0.0); // Way outside
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            contacts.is_empty(),
            "Outside bounds should produce 0 contacts"
        );
    }

    // ========================================================================
    // T8: Mesh with no convex hull → AC8
    // ========================================================================
    #[test]
    fn t8_mesh_no_hull_no_contact() {
        // A mesh geom with no mesh data → 0 contacts, no panic.
        // geom_mesh[1] = None → `CcdObj::of_geom` makes no object
        let model = flat_hfield_model(
            5,
            2.0,
            2.0,
            0.0,
            GeomType::Mesh,
            Vector3::new(0.5, 0.5, 0.5),
            1.0,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        let pos2 = Vector3::new(0.0, 0.0, 0.0);
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(contacts.is_empty());
    }

    // ========================================================================
    // T11: Hfield-plane returns no contacts → AC11
    // ========================================================================
    #[test]
    fn t11_hfield_plane_no_contact() {
        // `CcdObj::of_geom` takes no plane → 0 contacts
        let model = flat_hfield_model(
            5,
            2.0,
            2.0,
            0.0,
            GeomType::Plane,
            Vector3::new(1.0, 1.0, 0.0),
            f64::INFINITY,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        let pos2 = Vector3::new(0.0, 0.0, 0.0);
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            contacts.is_empty(),
            "Hfield-plane should produce 0 contacts"
        );
    }

    // ========================================================================
    // TS1: Single-cell hfield (2×2 grid)
    // ========================================================================
    #[test]
    fn ts1_single_cell_hfield() {
        // Minimum valid: 2×2 = 1 cell = 2 prisms
        let model = flat_hfield_model(
            2,
            1.0,
            1.0,
            0.5,
            GeomType::Sphere,
            Vector3::new(0.3, 0.0, 0.0),
            0.3,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        let pos2 = Vector3::new(0.0, 0.0, 0.7); // Sphere bottom at 0.4, terrain at 0.5
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            !contacts.is_empty(),
            "Single-cell hfield should still produce contacts"
        );
    }

    // ========================================================================
    // TS2: Edge-straddling geom
    // ========================================================================
    #[test]
    fn ts2_edge_straddling() {
        // Sphere at the edge of the hfield
        let model = flat_hfield_model(
            5,
            2.0,
            2.0,
            0.5,
            GeomType::Sphere,
            Vector3::new(0.5, 0.0, 0.0),
            0.5,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        // Sphere center at x=1.8 (near edge at x=2.0), z=0.9 → bottom at 0.4
        let pos2 = Vector3::new(1.8, 0.0, 0.9);
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            !contacts.is_empty(),
            "Edge-straddling sphere should produce contacts"
        );
    }

    // ========================================================================
    // TS3: Small geom in single cell — sub-grid converges to 1-2 prisms
    // ========================================================================
    #[test]
    fn ts3_small_geom_one_cell() {
        // 10×10 hfield with tiny sphere fitting within a single cell.
        // Cell size = 2*5/(10-1) ≈ 1.11. Sphere radius=0.1, well within one cell.
        let model = flat_hfield_model(
            10,
            5.0,
            5.0,
            0.5,
            GeomType::Sphere,
            Vector3::new(0.1, 0.0, 0.0),
            0.1,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        // Sphere at origin, z=0.55 → bottom at 0.45, terrain at 0.5 → penetrating
        let pos2 = Vector3::new(0.0, 0.0, 0.55);
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            !contacts.is_empty(),
            "Tiny sphere in single cell should produce contacts"
        );
        // Tiny sphere should only hit 1-2 prisms → few contacts
        assert!(
            contacts.len() <= 4,
            "Tiny sphere should hit at most a few prisms, got {}",
            contacts.len()
        );
    }

    // ========================================================================
    // TS4: Tilted hfield (non-identity pose)
    // ========================================================================
    #[test]
    fn ts4_tilted_hfield() {
        // Hfield rotated 45 degrees around Y axis
        let model = flat_hfield_model(
            5,
            2.0,
            2.0,
            0.5,
            GeomType::Sphere,
            Vector3::new(0.3, 0.0, 0.0),
            0.3,
        );
        let angle = std::f64::consts::FRAC_PI_4; // 45 degrees
        let hf_mat = Matrix3::new(
            angle.cos(),
            0.0,
            angle.sin(),
            0.0,
            1.0,
            0.0,
            -angle.sin(),
            0.0,
            angle.cos(),
        );
        let pos1 = Vector3::zeros();
        let pos2 = Vector3::new(0.0, 0.0, 0.7);
        let mat2 = Matrix3::identity();

        // Should not panic with non-identity hfield rotation
        let contacts = collide_hfield_multi(&model, 0, 1, pos1, hf_mat, pos2, mat2, 0.0);
        // Whether contacts are generated depends on geometry after rotation,
        // but no panic is the key assertion.
        let _ = contacts;
    }

    // ========================================================================
    // TS5: Flat prism (all heights same = degenerate slab)
    // ========================================================================
    #[test]
    fn ts5_flat_prism_degenerate() {
        // All heights identical — prism top is flat. GJK should still work.
        let model = flat_hfield_model(
            3,
            1.0,
            1.0,
            1.0, // flat at z=1.0
            GeomType::Sphere,
            Vector3::new(0.3, 0.0, 0.0),
            0.3,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        let pos2 = Vector3::new(0.0, 0.0, 1.2); // Sphere bottom at 0.9, terrain at 1.0
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            !contacts.is_empty(),
            "Flat prism should still produce contacts"
        );
    }

    // ========================================================================
    // TS6: Hfield-hfield (no crash, graceful)
    // ========================================================================
    #[test]
    fn ts6_hfield_hfield_no_crash() {
        // Two hfield geoms — 0 contacts (`CcdObj::of_geom` takes no height field)
        let heights = vec![0.5; 25];
        let cell_size = 1.0;
        let hfield = HeightFieldData::new(heights, 5, 5, cell_size);

        let mut model = Model::empty();
        model.ngeom = 2;
        model.nbody = 2;
        model.body_parent.push(0);
        model.body_rootid.push(1);
        model.body_jnt_adr.push(0);
        model.body_jnt_num.push(0);
        model.body_dof_adr.push(0);
        model.body_dof_num.push(0);
        model.body_geom_adr.push(1);
        model.body_geom_num.push(1);
        model.body_pos.push(Vector3::zeros());
        model.body_quat.push(UnitQuaternion::identity());
        model.body_ipos.push(Vector3::zeros());
        model.body_iquat.push(UnitQuaternion::identity());
        model.body_mass.push(0.0);
        model.body_inertia.push(Vector3::zeros());
        model.body_name.push(None);
        model.body_subtreemass.push(0.0);
        model.body_mocapid.push(None);
        model.body_gravcomp.push(0.0);
        model.body_invweight0.push([0.0; 2]);
        model.body_treeid.push(0);

        model.geom_type = vec![GeomType::Hfield; 2];
        model.geom_body = vec![0, 1];
        model.geom_pos = vec![Vector3::zeros(); 2];
        model.geom_quat = vec![UnitQuaternion::identity(); 2];
        model.geom_size = vec![Vector3::zeros(); 2];
        model.geom_name = vec![None; 2];
        model.geom_rbound = vec![f64::INFINITY; 2];
        model.geom_mesh = vec![None; 2];
        model.geom_hfield = vec![Some(0), Some(0)];
        model.geom_shape = vec![None; 2];
        model.geom_contype = vec![1; 2];
        model.geom_conaffinity = vec![1; 2];
        model.geom_friction = vec![Vector3::new(1.0, 0.005, 0.0001); 2];
        model.geom_condim = vec![3; 2];
        model.geom_solref = vec![[0.02, 1.0]; 2];
        model.geom_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]; 2];
        model.geom_priority = vec![0; 2];
        model.geom_solmix = vec![1.0; 2];
        model.geom_margin = vec![0.0; 2];
        model.geom_gap = vec![0.0; 2];
        model.geom_group = vec![0; 2];
        model.geom_rgba = vec![[1.0; 4]; 2];
        model.geom_fluid = vec![[0.0; 12]; 2];

        model.hfield_data = vec![Arc::new(hfield)];
        model.hfield_size = vec![[2.0, 2.0, 1.0, 1.0]];

        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        let pos2 = Vector3::new(0.0, 0.0, 0.0);
        let mat2 = Matrix3::identity();

        // Hfield-hfield: the "convex" geom is also Hfield, which `CcdObj::of_geom` does not take → 0 contacts
        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            contacts.is_empty(),
            "Hfield-hfield should produce 0 contacts"
        );
    }

    // ========================================================================
    // Test: Swapped ordering (convex is geom1, hfield is geom2)
    // ========================================================================
    #[test]
    fn test_swapped_geom_order() {
        // Ensure contacts work when hfield is geom2 (not geom1); each is listed
        // with the height field first, its normal up from the field (MuJoCo's
        // type order).
        let model = flat_hfield_model(
            5,
            2.0,
            2.0,
            0.5,
            GeomType::Sphere,
            Vector3::new(0.3, 0.0, 0.0),
            0.3,
        );
        // Pass geom indices swapped: sphere=geom1(idx=1), hfield=geom2(idx=0)
        let pos_hf = Vector3::zeros();
        let mat_hf = Matrix3::identity();
        let pos_sphere = Vector3::new(0.0, 0.0, 0.7);
        let mat_sphere = Matrix3::identity();

        let contacts = collide_hfield_multi(
            &model, 1, 0, // sphere first, hfield second
            pos_sphere, mat_sphere, pos_hf, mat_hf, 0.0,
        );
        assert!(
            !contacts.is_empty(),
            "Swapped order should still produce contacts"
        );
        for c in &contacts {
            assert_eq!([c.geom1, c.geom2], [0, 1]);
            assert!(c.normal.z > 0.9, "normal {:?}", c.normal);
        }
    }

    // ========================================================================
    // Test: Hfield-box capsule contact (basic sanity for box type)
    // ========================================================================
    #[test]
    fn test_hfield_box_contact() {
        let model = flat_hfield_model(
            5,
            2.0,
            2.0,
            0.0,
            GeomType::Box,
            Vector3::new(0.5, 0.5, 0.5), // 1×1×1 box
            0.87,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        // Box center at z=0.3 → bottom at z=-0.2, terrain at z=0 → penetrating
        let pos2 = Vector3::new(0.0, 0.0, 0.3);
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            !contacts.is_empty(),
            "Box penetrating flat terrain should produce contacts"
        );
    }

    // ========================================================================
    // Test: Capsule on hfield
    // ========================================================================
    #[test]
    fn test_hfield_capsule_contact() {
        let model = flat_hfield_model(
            5,
            2.0,
            2.0,
            0.0,
            GeomType::Capsule,
            Vector3::new(0.3, 0.5, 0.0), // radius=0.3, half_length=0.5
            0.8,
        );
        let pos1 = Vector3::zeros();
        let mat1 = Matrix3::identity();
        // Standing capsule at z=0.2 → bottom hemisphere center at z=-0.3,
        // hemisphere bottom at z=-0.6 → penetrating terrain at z=0
        let pos2 = Vector3::new(0.0, 0.0, 0.2);
        let mat2 = Matrix3::identity();

        let contacts = collide_hfield_multi(&model, 0, 1, pos1, mat1, pos2, mat2, 0.0);
        assert!(
            !contacts.is_empty(),
            "Capsule penetrating terrain should produce contacts"
        );
    }

    /// A field of zero width, or of a width of 1e-300, with a sphere beside it
    /// (past its +x edge, within the margin): the sub-grid's first column lies
    /// far past the grid (its cell index saturates), and the port lists
    /// nothing, as MuJoCo's empty loop would, where `cmin + 1` once
    /// overflowed.
    #[test]
    fn a_field_of_no_width_collides_as_nothing() {
        for half in [0.0, 1e-300] {
            let mut model = flat_hfield_model(
                5,
                1.0,
                1.0,
                0.0,
                GeomType::Sphere,
                Vector3::new(0.1, 0.0, 0.0),
                0.1,
            );
            model.hfield_size[0][0] = half;
            let contacts = collide_hfield_multi(
                &model,
                0,
                1,
                Vector3::zeros(),
                Matrix3::identity(),
                Vector3::new(0.15, 0.0, 0.05),
                Matrix3::identity(),
                0.1,
            );
            assert!(contacts.is_empty(), "half-width {half}: {contacts:?}");
        }
    }

    /// A field whose heights pass its top (`size[2]`), as sim-mjcf loads
    /// elevations above 1 until book Rigid-loading L45 normalises them: a
    /// sphere resting on it above the top still collides, the box tests
    /// taking the field's highest point. MuJoCo's field never passes its top,
    /// so its box tests take `size[2]`.
    #[test]
    fn a_field_above_its_top_still_collides() {
        let model = flat_hfield_model(
            5,
            1.0,
            1.0,
            2.0,
            GeomType::Sphere,
            Vector3::new(0.1, 0.0, 0.0),
            0.1,
        );
        assert!(model.hfield_size[0][2] < 2.0);
        let contacts = collide_hfield_multi(
            &model,
            0,
            1,
            Vector3::zeros(),
            Matrix3::identity(),
            Vector3::new(0.1, 0.05, 2.09),
            Matrix3::identity(),
            0.0,
        );
        assert!(!contacts.is_empty(), "a sphere 0.01 into the field");
        assert!(contacts.iter().all(|c| c.depth > 0.0));
    }

    /// The ccd golden (`sim/L0/tests/scripts/gen_ccd_reference.py`).
    fn ccd_golden() -> serde_json::Value {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../tests/assets/golden/ccd/ccd.json"
        );
        let text = std::fs::read_to_string(path).expect("the ccd golden reads");
        serde_json::from_str(&text).expect("the ccd golden parses")
    }

    fn numbers(v: &serde_json::Value) -> Vec<f64> {
        v.as_array()
            .expect("an array")
            .iter()
            .map(|x| x.as_f64().expect("a number"))
            .collect()
    }

    fn vector(v: &serde_json::Value) -> Vector3<f64> {
        Vector3::from_column_slice(&numbers(v))
    }

    /// `mju_normalize3`.
    fn normalized(v: Vector3<f64>) -> Vector3<f64> {
        let norm = (v.x * v.x + v.y * v.y + v.z * v.z).sqrt();
        let inv = 1.0 / norm;
        Vector3::new(v.x * inv, v.y * inv, v.z * inv)
    }

    /// Book P52's must-fail: every height-field case of the ccd golden with
    /// a primitive (`hfield_convex` `pose` and `edge`, 249), through
    /// `collide_hfield_multi` on a model holding MuJoCo's field data scaled
    /// by its top (rows bottom to top, normalised: in code, as sim-mjcf does
    /// not load them so yet, book Rigid-loading L45) and MuJoCo's bounding
    /// radius: MuJoCo's contacts, the field first, each distance and
    /// position bit for bit and its normal as `mj_setContact` normalises
    /// MuJoCo's (`mju_makeFrame`).
    #[test]
    fn hfield_contacts_match_mujoco_bitwise() {
        let doc = ccd_golden();
        let mut total = 0;
        for group in ["pose", "edge"] {
            for (name, case) in doc["hfield_convex"][group].as_object().expect("cases") {
                let kind = case["type"].as_str().expect("a type");
                let conv_type = match kind {
                    "sphere" => GeomType::Sphere,
                    "capsule" => GeomType::Capsule,
                    "ellipsoid" => GeomType::Ellipsoid,
                    "cylinder" => GeomType::Cylinder,
                    "box" => GeomType::Box,
                    _ => continue,
                };
                let field = &doc["hfield_fields"][case["field"].as_str().expect("a field")];
                let size = numbers(&field["size"]);
                let heights = numbers(&field["data"])
                    .iter()
                    .map(|h| h * size[2])
                    .collect();
                let count =
                    |v: &serde_json::Value| usize::try_from(v.as_u64().expect("n")).expect("n");
                let mut model = make_hfield_model(
                    heights,
                    count(&field["ncol"]),
                    count(&field["nrow"]),
                    [size[0], size[1], size[2], size[3]],
                    conv_type,
                    vector(&case["size"]),
                    case["rbound"].as_f64().expect("rbound"),
                );
                model.ccd_iterations = count(&case["ccd"][0]);
                model.ccd_tolerance = case["ccd"][1].as_f64().expect("tolerance");
                let contacts = collide_hfield_multi(
                    &model,
                    0,
                    1,
                    vector(&case["hfield"]["pos"]),
                    Matrix3::from_row_slice(&numbers(&case["hfield"]["mat"])),
                    vector(&case["pos"]),
                    Matrix3::from_row_slice(&numbers(&case["mat"])),
                    case["margin"].as_f64().expect("margin"),
                );
                let want = case["contacts"].as_array().expect("contacts");
                assert_eq!(contacts.len(), want.len(), "{name}: contacts");
                for (k, (c, w)) in contacts.iter().zip(want).enumerate() {
                    assert_eq!([c.geom1, c.geom2], [0, 1], "{name} {k}: order");
                    let dist = w["dist"].as_f64().expect("dist");
                    assert_eq!((-c.depth).to_bits(), dist.to_bits(), "{name} {k}: dist");
                    assert_eq!(c.pos, vector(&w["pos"]), "{name} {k}: position");
                    assert_eq!(
                        c.normal,
                        normalized(vector(&w["normal"])),
                        "{name} {k}: normal"
                    );
                }
                total += 1;
            }
        }
        assert_eq!(total, 249, "primitive cases");
    }
}
