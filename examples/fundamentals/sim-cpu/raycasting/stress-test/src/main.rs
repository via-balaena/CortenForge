//! Stress test — headless validation of ray intersection against analytical solutions.
//!
//! 20 checks covering: per-shape hit/miss (10), surface normal properties (3),
//! scene-level queries (4), and edge cases (3).
//!
//! Run: `cargo run -p example-raycasting-stress-test --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::similar_names,
    clippy::too_many_lines,
    clippy::suboptimal_flops,
    clippy::needless_range_loop,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::option_if_let_else
)]

use std::sync::Arc;

use nalgebra::{Matrix3, Point3, UnitQuaternion, UnitVector3, Vector3};

use cf_geometry::{HeightFieldData, Shape};
use sim_core::{Data, GeomType, Model, RaycastHit, raycast_scene, raycast_shape};
use sim_types::Pose;

// ── Helpers ────────────────────────────────────────────────────────────────

fn check(name: &str, pass: bool, detail: &str) -> bool {
    let tag = if pass { "PASS" } else { "FAIL" };
    println!("  [{tag}] {name}: {detail}");
    pass
}

/// Standard ray origin and +Z direction used by most shape checks.
fn origin_and_plus_z() -> (Point3<f64>, UnitVector3<f64>) {
    (Point3::origin(), UnitVector3::new_normalize(Vector3::z()))
}

/// Compute all per-shape hits for reuse by normal-property checks.
/// Returns `(shape_name, ray_direction, Option<RaycastHit>)` for each shape test that should hit.
fn all_shape_hits() -> Vec<(&'static str, Vector3<f64>, RaycastHit)> {
    let (origin, dir_z) = origin_and_plus_z();
    let max_dist = 20.0;

    let cases: Vec<(&str, Shape, Pose, Point3<f64>, UnitVector3<f64>)> = vec![
        // 1. Sphere
        (
            "sphere",
            Shape::sphere(1.0),
            Pose::from_position(Point3::new(0.0, 0.0, 5.0)),
            origin,
            dir_z,
        ),
        // 3. Plane
        (
            "plane",
            Shape::plane(Vector3::z(), 0.0),
            Pose::from_position(Point3::new(0.0, 0.0, 3.0)),
            origin,
            dir_z,
        ),
        // 5. Box
        (
            "box",
            Shape::box_shape(Vector3::new(1.0, 1.0, 1.0)),
            Pose::from_position(Point3::new(0.0, 0.0, 5.0)),
            origin,
            dir_z,
        ),
        // 6. Capsule
        (
            "capsule",
            Shape::capsule(1.0, 0.3),
            Pose::from_position(Point3::new(0.0, 0.0, 5.0)),
            origin,
            dir_z,
        ),
        // 7. Cylinder
        (
            "cylinder",
            Shape::cylinder(1.0, 0.5),
            Pose::from_position(Point3::new(0.0, 0.0, 5.0)),
            origin,
            dir_z,
        ),
        // 8. Ellipsoid
        (
            "ellipsoid",
            Shape::ellipsoid(Vector3::new(1.0, 1.0, 2.0)),
            Pose::from_position(Point3::new(0.0, 0.0, 5.0)),
            origin,
            dir_z,
        ),
    ];

    let mut hits = Vec::new();
    for (name, shape, pose, o, d) in &cases {
        if let Some(hit) = raycast_shape(shape, pose, *o, *d, max_dist) {
            hits.push((*name, d.into_inner(), hit));
        }
    }
    hits
}

/// Push a sphere geom into a manually-built Model.
fn push_sphere_geom(model: &mut Model, body_id: usize, radius: f64) {
    model.geom_type.push(GeomType::Sphere);
    model.geom_body.push(body_id);
    model.geom_pos.push(Vector3::zeros());
    model.geom_quat.push(UnitQuaternion::identity());
    model.geom_size.push(Vector3::new(radius, radius, radius));
    model.geom_friction.push(Vector3::new(1.0, 0.005, 0.0001));
    model.geom_condim.push(3);
    model.geom_contype.push(1);
    model.geom_conaffinity.push(1);
    model.geom_margin.push(0.0);
    model.geom_gap.push(0.0);
    model.geom_priority.push(0);
    model.geom_solmix.push(1.0);
    model.geom_solimp.push([0.9, 0.95, 0.001, 0.5, 2.0]);
    model.geom_solref.push([0.02, 1.0]);
    model.geom_name.push(None);
    model.geom_rbound.push(radius);
    model.geom_mesh.push(None);
    model.geom_hfield.push(None);
    model.geom_shape.push(None);
    model.geom_group.push(0);
    model.geom_rgba.push([0.5, 0.5, 0.5, 1.0]);
    model.ngeom += 1;
}

/// Build a scene with two spheres along +Z for scene-level checks.
/// Geom 0: sphere r=1 at z=5 (body 1). Geom 1: sphere r=1 at z=10 (body 2).
fn make_two_sphere_scene() -> (Model, Data) {
    let mut model = Model::empty();
    model.nbody = 3; // world + 2 bodies

    push_sphere_geom(&mut model, 1, 1.0);
    push_sphere_geom(&mut model, 2, 1.0);

    let mut data = model.make_data();
    data.geom_xpos = vec![Vector3::new(0.0, 0.0, 5.0), Vector3::new(0.0, 0.0, 10.0)];
    data.geom_xmat = vec![Matrix3::identity(); 2];

    (model, data)
}

// ── Check 1: Sphere hit distance ───────────────────────────────────────────

fn check_1_sphere_hit() -> (u32, u32) {
    let shape = Shape::sphere(1.0);
    let pose = Pose::from_position(Point3::new(0.0, 0.0, 5.0));
    let (origin, dir) = origin_and_plus_z();

    let hit = raycast_shape(&shape, &pose, origin, dir, 20.0);
    let p = match hit {
        Some(h) => {
            let err = (h.distance - 4.0).abs();
            check(
                "Sphere hit distance",
                err < 1e-6,
                &format!("dist = {:.6}, expected 4.0, err = {err:.2e}", h.distance),
            )
        }
        None => check("Sphere hit distance", false, "got None, expected hit"),
    };
    (u32::from(p), 1)
}

// ── Check 2: Sphere miss ───────────────────────────────────────────────────

fn check_2_sphere_miss() -> (u32, u32) {
    let shape = Shape::sphere(1.0);
    let pose = Pose::from_position(Point3::new(0.0, 0.0, 5.0));
    let origin = Point3::origin();
    let dir = UnitVector3::new_normalize(Vector3::x());

    let hit = raycast_shape(&shape, &pose, origin, dir, 20.0);
    let p = check("Sphere miss", hit.is_none(), "ray in +X at sphere at z=5");
    (u32::from(p), 1)
}

// ── Check 3: Plane hit distance ────────────────────────────────────────────

fn check_3_plane_hit() -> (u32, u32) {
    let shape = Shape::plane(Vector3::z(), 0.0);
    let pose = Pose::from_position(Point3::new(0.0, 0.0, 3.0));
    let (origin, dir) = origin_and_plus_z();

    let hit = raycast_shape(&shape, &pose, origin, dir, 20.0);
    let p = match hit {
        Some(h) => {
            let err = (h.distance - 3.0).abs();
            check(
                "Plane hit distance",
                err < 1e-6,
                &format!("dist = {:.6}, expected 3.0, err = {err:.2e}", h.distance),
            )
        }
        None => check("Plane hit distance", false, "got None, expected hit"),
    };
    (u32::from(p), 1)
}

// ── Check 4: Plane parallel miss ───────────────────────────────────────────

fn check_4_plane_parallel_miss() -> (u32, u32) {
    let shape = Shape::plane(Vector3::z(), 0.0);
    let pose = Pose::from_position(Point3::new(0.0, 0.0, 3.0));
    let origin = Point3::origin();
    let dir = UnitVector3::new_normalize(Vector3::x());

    let hit = raycast_shape(&shape, &pose, origin, dir, 20.0);
    let p = check(
        "Plane parallel miss",
        hit.is_none(),
        "ray in +X at plane with normal +Z",
    );
    (u32::from(p), 1)
}

// ── Check 5: Box slab intersection ─────────────────────────────────────────

fn check_5_box_hit() -> (u32, u32) {
    let shape = Shape::box_shape(Vector3::new(1.0, 1.0, 1.0));
    let pose = Pose::from_position(Point3::new(0.0, 0.0, 5.0));
    let (origin, dir) = origin_and_plus_z();

    let hit = raycast_shape(&shape, &pose, origin, dir, 20.0);
    let p = match hit {
        Some(h) => {
            let dist_err = (h.distance - 4.0).abs();
            let nz_err = (h.normal.z - (-1.0)).abs();
            let ok = dist_err < 1e-6 && nz_err < 1e-6;
            check(
                "Box slab intersection",
                ok,
                &format!(
                    "dist = {:.6}, normal.z = {:.6} (expected 4.0, -1.0)",
                    h.distance, h.normal.z
                ),
            )
        }
        None => check("Box slab intersection", false, "got None, expected hit"),
    };
    (u32::from(p), 1)
}

// ── Check 6: Capsule sphere-swept-line ─────────────────────────────────────

fn check_6_capsule_hit() -> (u32, u32) {
    let shape = Shape::capsule(1.0, 0.3);
    let pose = Pose::from_position(Point3::new(0.0, 0.0, 5.0));
    let (origin, dir) = origin_and_plus_z();

    let hit = raycast_shape(&shape, &pose, origin, dir, 20.0);
    let p = match hit {
        Some(h) => {
            let err = (h.distance - 3.7).abs();
            check(
                "Capsule hit (bottom cap)",
                err < 1e-3,
                &format!("dist = {:.6}, expected 3.7, err = {err:.2e}", h.distance),
            )
        }
        None => check("Capsule hit (bottom cap)", false, "got None, expected hit"),
    };
    (u32::from(p), 1)
}

// ── Check 7: Cylinder flat cap ─────────────────────────────────────────────

fn check_7_cylinder_hit() -> (u32, u32) {
    let shape = Shape::cylinder(1.0, 0.5);
    let pose = Pose::from_position(Point3::new(0.0, 0.0, 5.0));
    let (origin, dir) = origin_and_plus_z();

    let hit = raycast_shape(&shape, &pose, origin, dir, 20.0);
    let p = match hit {
        Some(h) => {
            let err = (h.distance - 4.0).abs();
            check(
                "Cylinder flat cap",
                err < 1e-6,
                &format!("dist = {:.6}, expected 4.0, err = {err:.2e}", h.distance),
            )
        }
        None => check("Cylinder flat cap", false, "got None, expected hit"),
    };
    (u32::from(p), 1)
}

// ── Check 8: Ellipsoid scaled sphere ───────────────────────────────────────

fn check_8_ellipsoid_hit() -> (u32, u32) {
    let shape = Shape::ellipsoid(Vector3::new(1.0, 1.0, 2.0));
    let pose = Pose::from_position(Point3::new(0.0, 0.0, 5.0));
    let (origin, dir) = origin_and_plus_z();

    let hit = raycast_shape(&shape, &pose, origin, dir, 20.0);
    let p = match hit {
        Some(h) => {
            let err = (h.distance - 3.0).abs();
            check(
                "Ellipsoid scaled sphere",
                err < 1e-3,
                &format!("dist = {:.6}, expected 3.0, err = {err:.2e}", h.distance),
            )
        }
        None => check("Ellipsoid scaled sphere", false, "got None, expected hit"),
    };
    (u32::from(p), 1)
}

// ── Check 9: Heightfield ray march ─────────────────────────────────────────

fn check_9_heightfield_hit() -> (u32, u32) {
    let hfield = HeightFieldData::flat(10, 10, 1.0, 0.0);
    let shape = Shape::height_field(Arc::new(hfield));
    let pose = Pose::identity();
    let origin = Point3::new(4.0, 4.0, 3.0); // center of field
    let dir = UnitVector3::new_normalize(-Vector3::z());

    let hit = raycast_shape(&shape, &pose, origin, dir, 20.0);
    let p = match hit {
        Some(h) => {
            let dist_err = (h.distance - 3.0).abs();
            let z_err = h.point.z.abs();
            let ok = dist_err < 0.01 && z_err < 0.01;
            check(
                "Heightfield ray march",
                ok,
                &format!(
                    "dist = {:.4}, hit.z = {:.4} (expected 3.0, 0.0)",
                    h.distance, h.point.z
                ),
            )
        }
        None => check("Heightfield ray march", false, "got None, expected hit"),
    };
    (u32::from(p), 1)
}

// ── Check 10: Convex mesh face test ────────────────────────────────────────

fn check_10_convex_mesh() -> (u32, u32) {
    let vertices = vec![
        Point3::new(0.0, 0.0, -2.0),
        Point3::new(0.1, 0.0, 2.0),
        Point3::new(-0.05, 0.087, 2.0),
        Point3::new(-0.05, -0.087, 2.0),
    ];
    let hull = cf_geometry::convex_hull(&vertices, None).expect("convex hull");
    let shape = Shape::convex_mesh(hull);
    let pose = Pose::from_position(Point3::new(0.0, 0.0, 5.0));
    let dir = UnitVector3::new_normalize(Vector3::z());

    // Ray at center — should hit
    let hit_center = raycast_shape(&shape, &pose, Point3::origin(), dir, 20.0);
    let center_ok = hit_center.is_some();

    // Ray at x=0.8 — outside hull, inside bounding sphere — should miss
    let hit_wide = raycast_shape(&shape, &pose, Point3::new(0.8, 0.0, 0.0), dir, 20.0);
    let wide_ok = hit_wide.is_none();

    let p = check(
        "Convex mesh (hit center, miss outside hull)",
        center_ok && wide_ok,
        &format!("center_hit = {center_ok}, wide_miss = {wide_ok}"),
    );
    (u32::from(p), 1)
}

// ── Check 11: Normal is unit-length ────────────────────────────────────────

fn check_11_normal_unit_length() -> (u32, u32) {
    let hits = all_shape_hits();
    let mut all_ok = true;
    let mut worst_name = hits.first().map_or("(none)", |h| h.0);
    let mut worst_err = 0.0_f64;

    for (name, _, hit) in &hits {
        let norm = hit.normal.norm();
        let err = (norm - 1.0).abs();
        if err > worst_err {
            worst_err = err;
            worst_name = name;
        }
        if err > 0.001 {
            all_ok = false;
        }
    }

    let p = check(
        "Normal is unit-length (all shapes)",
        all_ok,
        &format!(
            "{} hits checked, worst: {} (err = {worst_err:.2e})",
            hits.len(),
            worst_name,
        ),
    );
    (u32::from(p), 1)
}

// ── Check 12: Normal faces ray ─────────────────────────────────────────────

fn check_12_normal_faces_ray() -> (u32, u32) {
    let hits = all_shape_hits();
    let mut all_ok = true;
    let mut worst_name = hits.first().map_or("(none)", |h| h.0);
    let mut worst_dot = f64::NEG_INFINITY;

    for (name, ray_dir, hit) in &hits {
        let dot = hit.normal.dot(ray_dir);
        if dot > worst_dot {
            worst_dot = dot;
            worst_name = name;
        }
        if dot >= 0.0 {
            all_ok = false;
        }
    }

    let p = check(
        "Normal faces ray (all shapes)",
        all_ok,
        &format!(
            "{} hits checked, worst dot: {} = {worst_dot:.6}",
            hits.len(),
            worst_name,
        ),
    );
    (u32::from(p), 1)
}

// ── Check 13: Normal perpendicular to surface ──────────────────────────────

fn check_13_normal_perpendicular() -> (u32, u32) {
    let (origin, dir) = origin_and_plus_z();
    let max_dist = 20.0;
    let mut passed = 0u32;
    let mut total = 0u32;

    // Sphere: normal = normalize(hit_point - center)
    {
        let center = Point3::new(0.0, 0.0, 5.0);
        let shape = Shape::sphere(1.0);
        let pose = Pose::from_position(center);
        if let Some(h) = raycast_shape(&shape, &pose, origin, dir, max_dist) {
            let expected = (h.point - center).normalize();
            let err = (h.normal - expected).norm();
            total += 1;
            if err < 1e-6 {
                passed += 1;
            }
            println!("    sphere: normal err = {err:.2e} (expected = normalize(hit - center))");
        }
    }

    // Plane: normal = ±plane_normal
    {
        let shape = Shape::plane(Vector3::z(), 0.0);
        let pose = Pose::from_position(Point3::new(0.0, 0.0, 3.0));
        if let Some(h) = raycast_shape(&shape, &pose, origin, dir, max_dist) {
            // Normal should be -Z (flipped to face incoming +Z ray)
            let err = (h.normal.z.abs() - 1.0).abs();
            total += 1;
            if err < 1e-6 {
                passed += 1;
            }
            println!("    plane: normal.z = {:.6} (expected ±1.0)", h.normal.z);
        }
    }

    // Box: normal aligned with face axis
    {
        let shape = Shape::box_shape(Vector3::new(1.0, 1.0, 1.0));
        let pose = Pose::from_position(Point3::new(0.0, 0.0, 5.0));
        if let Some(h) = raycast_shape(&shape, &pose, origin, dir, max_dist) {
            // Hit bottom face → normal should be (0,0,-1)
            let err = (h.normal - Vector3::new(0.0, 0.0, -1.0)).norm();
            total += 1;
            if err < 1e-6 {
                passed += 1;
            }
            println!(
                "    box: normal = ({:.4}, {:.4}, {:.4}), expected (0,0,-1)",
                h.normal.x, h.normal.y, h.normal.z
            );
        }
    }

    let all_ok = passed == total;
    let p = check(
        "Normal perpendicular to surface",
        all_ok,
        &format!("{passed}/{total} sub-checks"),
    );
    (u32::from(p), 1)
}

// ── Check 14: Scene nearest hit ────────────────────────────────────────────

fn check_14_scene_nearest() -> (u32, u32) {
    let (model, data) = make_two_sphere_scene();
    let (origin, dir) = origin_and_plus_z();

    let result = raycast_scene(&model, &data, origin, dir, 100.0, None, None);
    let p = match result {
        Some(h) => {
            let dist_ok = (h.hit.distance - 4.0).abs() < 1e-4;
            let id_ok = h.geom_id == 0;
            check(
                "Scene nearest hit",
                dist_ok && id_ok,
                &format!(
                    "geom_id = {}, dist = {:.4} (expected 0, 4.0)",
                    h.geom_id, h.hit.distance
                ),
            )
        }
        None => check("Scene nearest hit", false, "got None"),
    };
    (u32::from(p), 1)
}

// ── Check 15: Body exclude ─────────────────────────────────────────────────

fn check_15_body_exclude() -> (u32, u32) {
    let (model, data) = make_two_sphere_scene();
    let (origin, dir) = origin_and_plus_z();

    // Exclude body 1 (has geom 0) → should hit geom 1 at z=10
    let result = raycast_scene(&model, &data, origin, dir, 100.0, Some(1), None);
    let p = match result {
        Some(h) => {
            let dist_ok = (h.hit.distance - 9.0).abs() < 1e-4;
            let id_ok = h.geom_id == 1;
            check(
                "Body exclude",
                dist_ok && id_ok,
                &format!(
                    "geom_id = {}, dist = {:.4} (expected 1, 9.0)",
                    h.geom_id, h.hit.distance
                ),
            )
        }
        None => check("Body exclude", false, "got None"),
    };
    (u32::from(p), 1)
}

// ── Check 16: Geom group filter ────────────────────────────────────────────

fn check_16_geomgroup_filter() -> (u32, u32) {
    let (mut model, mut data) = make_two_sphere_scene();

    // Geom 0 → group 1, geom 1 → group 0
    model.geom_group[0] = 1;
    model.geom_group[1] = 0;

    // Re-set positions after model mutation
    data.geom_xpos = vec![Vector3::new(0.0, 0.0, 5.0), Vector3::new(0.0, 0.0, 10.0)];
    data.geom_xmat = vec![Matrix3::identity(); 2];

    let (origin, dir) = origin_and_plus_z();

    // Only include group 0 → geom 0 (group 1) filtered out
    let groups = [true, false, false, false, false, false];
    let result = raycast_scene(&model, &data, origin, dir, 100.0, None, Some(&groups));
    let p = match result {
        Some(h) => {
            let id_ok = h.geom_id == 1;
            check(
                "Geom group filter",
                id_ok,
                &format!("geom_id = {} (expected 1, the group-0 sphere)", h.geom_id),
            )
        }
        None => check("Geom group filter", false, "got None"),
    };
    (u32::from(p), 1)
}

// ── Check 17: Scene miss ───────────────────────────────────────────────────

fn check_17_scene_miss() -> (u32, u32) {
    let (model, data) = make_two_sphere_scene();
    let origin = Point3::origin();
    let dir = UnitVector3::new_normalize(-Vector3::z()); // away from spheres

    let result = raycast_scene(&model, &data, origin, dir, 100.0, None, None);
    let p = check(
        "Scene miss (ray -Z)",
        result.is_none(),
        "ray away from all geoms",
    );
    (u32::from(p), 1)
}

// ── Check 18: Ray origin inside sphere ─────────────────────────────────────

fn check_18_inside_sphere() -> (u32, u32) {
    let shape = Shape::sphere(1.0);
    let pose = Pose::identity(); // sphere at origin
    let origin = Point3::origin(); // ray starts at sphere center
    let dir = UnitVector3::new_normalize(Vector3::z());

    let hit = raycast_shape(&shape, &pose, origin, dir, 20.0);
    let p = match hit {
        Some(h) => {
            let err = (h.distance - 1.0).abs();
            check(
                "Inside sphere → exit point",
                err < 1e-6,
                &format!("dist = {:.6}, expected 1.0 (= radius)", h.distance),
            )
        }
        None => check(
            "Inside sphere → exit point",
            false,
            "got None, expected exit hit",
        ),
    };
    (u32::from(p), 1)
}

// ── Check 19: Max distance clamp ───────────────────────────────────────────

fn check_19_max_distance_clamp() -> (u32, u32) {
    let shape = Shape::sphere(1.0);
    let pose = Pose::from_position(Point3::new(0.0, 0.0, 10.0));
    let (origin, dir) = origin_and_plus_z();

    // Hit would be at dist=9, but max_distance=5 → None
    let hit = raycast_shape(&shape, &pose, origin, dir, 5.0);
    let p = check(
        "Max distance clamp",
        hit.is_none(),
        "sphere at z=10, max_dist=5 → hit at 9 is beyond cutoff",
    );
    (u32::from(p), 1)
}

// ── Check 20: Zero max distance ────────────────────────────────────────────

fn check_20_zero_max_distance() -> (u32, u32) {
    let shape = Shape::sphere(1.0);
    let pose = Pose::from_position(Point3::new(0.0, 0.0, 0.5));
    let (origin, dir) = origin_and_plus_z();

    let hit = raycast_shape(&shape, &pose, origin, dir, 0.0);
    let p = check(
        "Zero max distance → None",
        hit.is_none(),
        "max_distance = 0.0",
    );
    (u32::from(p), 1)
}

// ── Main ───────────────────────────────────────────────────────────────────

fn main() {
    println!("=== Raycasting — Stress Test ===\n");

    let checks: Vec<(&str, fn() -> (u32, u32))> = vec![
        ("Sphere hit distance", check_1_sphere_hit),
        ("Sphere miss", check_2_sphere_miss),
        ("Plane hit distance", check_3_plane_hit),
        ("Plane parallel miss", check_4_plane_parallel_miss),
        ("Box slab intersection", check_5_box_hit),
        ("Capsule sphere-swept-line", check_6_capsule_hit),
        ("Cylinder flat cap", check_7_cylinder_hit),
        ("Ellipsoid scaled sphere", check_8_ellipsoid_hit),
        ("Heightfield ray march", check_9_heightfield_hit),
        ("Convex mesh face test", check_10_convex_mesh),
        ("Normal is unit-length", check_11_normal_unit_length),
        ("Normal faces ray", check_12_normal_faces_ray),
        ("Normal perpendicular", check_13_normal_perpendicular),
        ("Scene nearest hit", check_14_scene_nearest),
        ("Body exclude", check_15_body_exclude),
        ("Geom group filter", check_16_geomgroup_filter),
        ("Scene miss", check_17_scene_miss),
        ("Inside sphere", check_18_inside_sphere),
        ("Max distance clamp", check_19_max_distance_clamp),
        ("Zero max distance", check_20_zero_max_distance),
    ];

    let mut total = 0u32;
    let mut passed = 0u32;

    for (i, (label, func)) in checks.iter().enumerate() {
        println!("-- {}. {} --", i + 1, label);
        let (p, t) = func();
        passed += p;
        total += t;
        println!();
    }

    println!("============================================================");
    println!("  TOTAL: {passed}/{total} checks passed");
    if passed == total {
        println!("  ALL PASS");
    } else {
        println!("  {} FAILED", total - passed);
        std::process::exit(1);
    }
}
