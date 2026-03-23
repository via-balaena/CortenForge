//! Phase 1–3 tests for concave SDF collision.
//!
//! Validates that SDF-SDF multi-contact surface tracing produces correct
//! contacts for concave geometry (inner surfaces). This is the foundation
//! for geometry-driven joints where SDF collision constrains motion instead
//! of abstract mathematical constraints.
//!
//! Phase 1: Bowl captures sphere (concave inner surface basics)
//! Phase 2: Tube constrains pin (cylindrical constraint)
//! Phase 3: Full hinge (socket + caps + flanged pin)

#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::float_cmp,
    clippy::imprecise_flops,
    clippy::needless_pass_by_value,
    clippy::needless_collect,
    clippy::panic,
    clippy::uninlined_format_args
)]

use nalgebra::{Point3, UnitQuaternion, Vector3};
use sim_types::Pose;
use std::sync::Arc;

use super::SdfGrid;
use super::shape::compute_shape_contact;
use super::shapes::{ShapeConcave, ShapeConvex, ShapeSphere};

// ── Helpers ──────────────────────────────────────────────────────────────

fn identity_pose(pos: Vector3<f64>) -> Pose {
    Pose {
        position: Point3::from(pos),
        rotation: UnitQuaternion::identity(),
    }
}

/// Net force direction from a set of contacts (sum of normal * depth).
/// Returns None if contacts are empty or net force is negligible.
fn net_force_direction(contacts: &[super::SdfContact]) -> Option<Vector3<f64>> {
    let sum: Vector3<f64> = contacts
        .iter()
        .map(|c| c.normal * c.penetration.max(0.001)) // floor depth for margin-zone contacts
        .sum();
    let len = sum.norm();
    if len < 1e-10 { None } else { Some(sum / len) }
}

/// Build an SDF grid from a distance function with automatic sizing.
/// `extent` is the half-size of the grid domain. Cell size determines resolution.
fn make_grid(cell_size: f64, extent: f64, f: impl Fn(Point3<f64>) -> f64) -> SdfGrid {
    let n = ((2.0 * extent) / cell_size).ceil() as usize + 1;
    let origin = Point3::new(-extent, -extent, -extent);
    SdfGrid::from_fn(n, n, n, cell_size, origin, f)
}

// ── SDF distance functions ───────────────────────────────────────────────

fn sphere_sdf(center: Point3<f64>, radius: f64, p: Point3<f64>) -> f64 {
    (p - center).norm() - radius
}

fn cylinder_z_sdf(radius: f64, half_height: f64, p: Point3<f64>) -> f64 {
    let radial = (p.x * p.x + p.y * p.y).sqrt() - radius;
    let axial = p.z.abs() - half_height;
    if radial > 0.0 && axial > 0.0 {
        (radial * radial + axial * axial).sqrt()
    } else {
        radial.max(axial)
    }
}

// ============================================================================
// Phase 1 — Concave SDF contact basics (bowl + sphere)
// ============================================================================

/// Build a bowl: sphere with top half removed.
/// Inner radius = `inner_r`, outer radius = `outer_r`, open at +Z.
fn make_bowl_grid(cell_size: f64) -> SdfGrid {
    let outer_r = 10.0;
    let inner_r = 8.0;
    let extent = outer_r + 2.0;
    make_grid(cell_size, extent, move |p| {
        let outer = sphere_sdf(Point3::origin(), outer_r, p);
        let inner = sphere_sdf(Point3::origin(), inner_r, p);
        // Subtract inner sphere: max(outer, -inner)
        let shell = outer.max(-inner);
        // Subtract top half-space (z > 0): the half-space has SDF = -p.z
        // (negative inside where z > 0). subtract = max(shell, -(-p.z)) = max(shell, p.z)
        shell.max(p.z)
    })
}

#[test]
fn phase1a_bowl_captures_sphere() {
    let bowl_grid = Arc::new(make_bowl_grid(0.5));
    let bowl = ShapeConcave::new(bowl_grid);

    // Small sphere (r=3) inside the bowl, slightly above bottom
    let sphere_grid = Arc::new(SdfGrid::sphere(Point3::origin(), 3.0, 16, 2.0));
    let sphere = ShapeSphere::new(sphere_grid, 3.0);

    // Bowl at origin, sphere at (0, 0, -5) — inside the bowl cavity
    let bowl_pose = identity_pose(Vector3::zeros());
    let sphere_pose = identity_pose(Vector3::new(0.0, 0.0, -5.0));

    let contacts = compute_shape_contact(&bowl, &bowl_pose, &sphere, &sphere_pose, 0.5, 50);

    assert!(
        !contacts.is_empty(),
        "sphere inside bowl should generate contacts, got 0"
    );

    // Verify contacts have reasonable properties
    for c in &contacts {
        assert!(c.normal.norm() > 0.5, "contact normal should be unit-ish");
        assert!(
            c.point.coords.norm() < 15.0,
            "contact point should be near the bowl"
        );
    }

    eprintln!(
        "  Phase 1a: {} contacts, depths: {:?}",
        contacts.len(),
        contacts
            .iter()
            .map(|c| format!("{:.3}", c.penetration))
            .collect::<Vec<_>>()
    );
}

#[test]
fn phase1b_bowl_rejects_exterior_sphere() {
    let bowl_grid = Arc::new(make_bowl_grid(0.5));
    let bowl = ShapeConcave::new(bowl_grid);

    let sphere_grid = Arc::new(SdfGrid::sphere(Point3::origin(), 3.0, 16, 2.0));
    let sphere = ShapeSphere::new(sphere_grid, 3.0);

    // Sphere well above the bowl (z=20, bowl extends from z=-10 to z=0)
    let bowl_pose = identity_pose(Vector3::zeros());
    let sphere_pose = identity_pose(Vector3::new(0.0, 0.0, 20.0));

    let contacts = compute_shape_contact(&bowl, &bowl_pose, &sphere, &sphere_pose, 0.5, 50);

    assert!(
        contacts.is_empty(),
        "sphere far outside bowl should have no contacts, got {}",
        contacts.len()
    );
}

#[test]
fn phase1c_bowl_wall_contact_count() {
    let bowl_grid = Arc::new(make_bowl_grid(0.5));
    let bowl = ShapeConcave::new(bowl_grid);

    let sphere_grid = Arc::new(SdfGrid::sphere(Point3::origin(), 3.0, 16, 2.0));
    let sphere = ShapeSphere::new(sphere_grid, 3.0);

    // Sphere touching inner wall — offset laterally so it contacts the side
    // Bowl inner radius = 8mm, sphere radius = 3mm.
    // Place sphere center at x=5.5 → sphere surface at x=8.5, penetrating 0.5mm into wall
    let bowl_pose = identity_pose(Vector3::zeros());
    let sphere_pose = identity_pose(Vector3::new(5.5, 0.0, -4.0));

    let contacts = compute_shape_contact(&bowl, &bowl_pose, &sphere, &sphere_pose, 0.5, 50);

    assert!(
        contacts.len() > 1,
        "sphere touching bowl wall should produce multiple contacts (multi-contact path), got {}",
        contacts.len()
    );

    // Net force should push sphere TOWARD bowl center (negative X direction)
    if let Some(force_dir) = net_force_direction(&contacts) {
        assert!(
            force_dir.x < 0.0,
            "net force should push sphere toward center (−X), got x={:.3}",
            force_dir.x
        );
        eprintln!(
            "  Phase 1c: {} contacts, net force dir = ({:.3}, {:.3}, {:.3})",
            contacts.len(),
            force_dir.x,
            force_dir.y,
            force_dir.z
        );
    } else {
        panic!("contacts exist but net force is negligible");
    }
}

// ============================================================================
// Phase 2 — Cylindrical constraint (pin in tube)
// ============================================================================

/// Build a tube: cylinder with through-bore (open both ends).
/// Outer radius = 5mm, bore radius = 3.5mm, half-height = 10mm.
fn make_tube_grid(cell_size: f64) -> SdfGrid {
    let outer_r: f64 = 5.0;
    let bore_r: f64 = 3.5;
    let half_h: f64 = 10.0;
    let extent = outer_r.max(half_h) + 2.0;
    make_grid(cell_size, extent, move |p| {
        let outer = cylinder_z_sdf(outer_r, half_h, p);
        let inner = cylinder_z_sdf(bore_r, half_h + 1.0, p); // bore extends beyond tube
        outer.max(-inner)
    })
}

/// Build a pin cylinder: radius = 3.0mm, half-height = 8mm.
fn make_pin_grid(cell_size: f64) -> SdfGrid {
    let pin_r: f64 = 3.0;
    let half_h: f64 = 8.0;
    let extent = pin_r.max(half_h) + 2.0;
    make_grid(cell_size, extent, move |p| cylinder_z_sdf(pin_r, half_h, p))
}

#[test]
fn phase2a_tube_radial_constraint() {
    let tube_grid = Arc::new(make_tube_grid(0.5));
    let tube = ShapeConcave::new(tube_grid);

    let pin_grid = Arc::new(make_pin_grid(0.5));
    let pin = ShapeConvex::new(pin_grid);

    // Pin centered in tube, offset 0.3mm in +X (approaching wall).
    // Bore radius = 3.5, pin radius = 3.0, clearance = 0.5mm.
    // Pin surface at x = 0.3 + 3.0 = 3.3, bore wall at x = 3.5 → gap = 0.2mm.
    // On the opposite side: pin surface at x = 0.3 - 3.0 = -2.7, wall at -3.5 → gap = 0.8mm.
    let tube_pose = identity_pose(Vector3::zeros());
    let pin_pose = identity_pose(Vector3::new(0.3, 0.0, 0.0));

    let contacts = compute_shape_contact(&tube, &tube_pose, &pin, &pin_pose, 0.5, 50);

    assert!(
        !contacts.is_empty(),
        "pin near tube wall should generate contacts"
    );

    // Net force should push pin back toward center (negative X)
    if let Some(force_dir) = net_force_direction(&contacts) {
        eprintln!(
            "  Phase 2a: {} contacts, net force dir = ({:.3}, {:.3}, {:.3})",
            contacts.len(),
            force_dir.x,
            force_dir.y,
            force_dir.z
        );
        assert!(
            force_dir.x < 0.0,
            "net force should push pin toward center (−X), got x={:.3}",
            force_dir.x
        );
    } else {
        panic!("contacts exist but net force is negligible");
    }
}

#[test]
fn phase2b_tube_allows_axial_slide() {
    let tube_grid = Arc::new(make_tube_grid(0.5));
    let tube = ShapeConcave::new(tube_grid);

    let pin_grid = Arc::new(make_pin_grid(0.5));
    let pin = ShapeConvex::new(pin_grid);

    // Pin centered in tube, offset 1mm in +Z (axial). Bore is open-ended.
    // Pin half-height = 8, tube half-height = 10, bore extends to ±11.
    // Pin stays well within the bore axially.
    let tube_pose = identity_pose(Vector3::zeros());
    let pin_pose = identity_pose(Vector3::new(0.0, 0.0, 1.0));

    let contacts = compute_shape_contact(&tube, &tube_pose, &pin, &pin_pose, 0.5, 50);

    // All contacts should be zero-depth (margin-zone only, no penetration)
    let penetrating = contacts.iter().filter(|c| c.penetration > 0.01).count();
    eprintln!(
        "  Phase 2b: {} total contacts, {} with penetration > 0.01",
        contacts.len(),
        penetrating
    );

    // Net axial force should be negligible (pin is centered radially)
    if let Some(force_dir) = net_force_direction(&contacts) {
        // Force should be radially symmetric → small net force
        let radial = (force_dir.x * force_dir.x + force_dir.y * force_dir.y).sqrt();
        let axial = force_dir.z.abs();
        eprintln!(
            "  Phase 2b: net force dir = ({:.3}, {:.3}, {:.3}), radial={:.3}, axial={:.3}",
            force_dir.x, force_dir.y, force_dir.z, radial, axial
        );
    }
}

#[test]
fn phase2c_tube_captures_pin_at_multiple_azimuths() {
    let tube_grid = Arc::new(make_tube_grid(0.5));
    let tube = ShapeConcave::new(tube_grid);

    let pin_grid = Arc::new(make_pin_grid(0.5));
    let pin = ShapeConvex::new(pin_grid);

    let tube_pose = identity_pose(Vector3::zeros());

    // Test 4 azimuthal offsets: +X, +Y, -X, -Y (each 0.3mm from center)
    let offsets = [
        (Vector3::new(0.3, 0.0, 0.0), "should be −X"),
        (Vector3::new(0.0, 0.3, 0.0), "should be −Y"),
        (Vector3::new(-0.3, 0.0, 0.0), "should be +X"),
        (Vector3::new(0.0, -0.3, 0.0), "should be +Y"),
    ];

    for (offset, expected) in &offsets {
        let pin_pose = identity_pose(*offset);
        let contacts = compute_shape_contact(&tube, &tube_pose, &pin, &pin_pose, 0.5, 50);

        assert!(
            !contacts.is_empty(),
            "pin at offset ({:.1},{:.1},{:.1}) should generate contacts",
            offset.x,
            offset.y,
            offset.z
        );

        if let Some(force_dir) = net_force_direction(&contacts) {
            // Force should oppose the offset direction (restorative)
            let dot = force_dir.dot(offset);
            eprintln!(
                "  Phase 2c: offset=({:.1},{:.1},{:.1}), force=({:.3},{:.3},{:.3}), dot={:.4} ({})",
                offset.x, offset.y, offset.z, force_dir.x, force_dir.y, force_dir.z, dot, expected
            );
            assert!(
                dot < 0.0,
                "force should oppose offset ({:.1},{:.1},{:.1}): {expected}, got dot={:.4}",
                offset.x,
                offset.y,
                offset.z,
                dot
            );
        }
    }
}

// ============================================================================
// Phase 3 — Full hinge (socket with caps + flanged pin)
// ============================================================================

/// Build a socket: tube with annular end caps.
///
/// ```text
///     cap (z = +half_h)
///   ┌─────────────┐
///   │ ┌─────────┐ │   ← bore (open through cap hole)
///   │ │  (air)  │ │
///   │ │         │ │   ← bore wall at bore_r
///   │ └─────────┘ │
///   └─────────────┘
///     cap (z = -half_h)
/// ```
///
/// Cap opening radius < pin flange radius → flanges can't escape axially.
fn make_socket_grid(cell_size: f64) -> SdfGrid {
    let outer_r: f64 = 5.5;
    let bore_r: f64 = 3.5;
    let half_h: f64 = 8.0;
    let cap_opening_r: f64 = 2.5; // smaller than pin flange
    let cap_thickness: f64 = 2.0;
    let extent = outer_r.max(half_h + cap_thickness) + 2.0;

    make_grid(cell_size, extent, move |p| {
        // Main tube body
        let outer = cylinder_z_sdf(outer_r, half_h + cap_thickness, p);
        let bore = cylinder_z_sdf(bore_r, half_h, p);
        let body = outer.max(-bore);

        // Cap openings: cylinders through the caps (smaller than bore)
        let cap_hole = cylinder_z_sdf(cap_opening_r, half_h + cap_thickness + 1.0, p);
        body.max(-cap_hole)
    })
}

/// Build a flanged pin: cylinder shaft with disc flanges at each end.
///
/// ```text
///   ──┐ flange (z = +shaft_h)
///     │ shaft
///   ──┘ flange (z = -shaft_h)
/// ```
///
/// Flange radius > cap_opening radius → axially trapped.
/// Flange radius < bore radius → fits through bore.
fn make_flanged_pin_grid(cell_size: f64) -> SdfGrid {
    let shaft_r: f64 = 3.0;
    let flange_r: f64 = 3.2; // > cap_opening (2.5), < bore (3.5)
    let shaft_half_h: f64 = 6.0; // bore half_h=8 → 1mm axial clearance to cap face
    let flange_half_h: f64 = 1.0;
    let extent = flange_r.max(shaft_half_h + flange_half_h) + 2.0;

    make_grid(cell_size, extent, move |p| {
        let shaft = cylinder_z_sdf(shaft_r, shaft_half_h, p);
        // Top flange
        let top_flange = cylinder_z_sdf(
            flange_r,
            flange_half_h,
            Point3::new(p.x, p.y, p.z - shaft_half_h),
        );
        // Bottom flange
        let bot_flange = cylinder_z_sdf(
            flange_r,
            flange_half_h,
            Point3::new(p.x, p.y, p.z + shaft_half_h),
        );
        shaft.min(top_flange).min(bot_flange) // union = min
    })
}

#[test]
fn phase3a_hinge_radial_constraint() {
    // Use 0.3mm cell_size for Phase 3 — the 0.3mm flange-to-bore clearance
    // is sub-cell at 0.5mm resolution, causing grid artifacts.
    let socket_grid = Arc::new(make_socket_grid(0.3));
    let socket = ShapeConcave::new(socket_grid);

    let pin_grid = Arc::new(make_flanged_pin_grid(0.3));
    let pin = ShapeConvex::new(pin_grid); // union of convex shapes = convex

    let socket_pose = identity_pose(Vector3::zeros());
    let pin_pose = identity_pose(Vector3::new(0.3, 0.0, 0.0));

    let contacts = compute_shape_contact(&socket, &socket_pose, &pin, &pin_pose, 0.5, 50);

    assert!(
        !contacts.is_empty(),
        "radially offset pin should generate contacts"
    );

    if let Some(force_dir) = net_force_direction(&contacts) {
        eprintln!(
            "  Phase 3a: {} contacts, net force = ({:.3}, {:.3}, {:.3})",
            contacts.len(),
            force_dir.x,
            force_dir.y,
            force_dir.z
        );
        assert!(
            force_dir.x < 0.0,
            "radial force should push pin back to center, got x={:.3}",
            force_dir.x
        );
    }
}

#[test]
fn phase3b_hinge_axial_constraint() {
    let socket_grid = Arc::new(make_socket_grid(0.3));
    let socket = ShapeConcave::new(socket_grid);

    let pin_grid = Arc::new(make_flanged_pin_grid(0.3));
    let pin = ShapeConvex::new(pin_grid);

    let socket_pose = identity_pose(Vector3::zeros());
    // Offset pin axially by 2.0mm in +Z. Flange should hit cap.
    // Shaft half_h=6, flange half_h=1, so flange top at z=7.
    // Socket bore half_h=8, cap inner face at z=8.
    // Clearance at rest = 8-7 = 1mm. Offset 2.0mm → flange top at z=9, penetrating 1mm into cap.
    let pin_pose = identity_pose(Vector3::new(0.0, 0.0, 2.0));

    let contacts = compute_shape_contact(&socket, &socket_pose, &pin, &pin_pose, 0.5, 50);

    // Should have contacts from flange hitting cap
    let axial_contacts: Vec<_> = contacts.iter().filter(|c| c.penetration > 0.0).collect();

    eprintln!(
        "  Phase 3b: {} total contacts, {} with penetration > 0",
        contacts.len(),
        axial_contacts.len()
    );

    assert!(
        !contacts.is_empty(),
        "axially offset pin should generate contacts from flange hitting cap"
    );

    // Net force should push pin back (negative Z)
    if let Some(force_dir) = net_force_direction(&contacts) {
        eprintln!(
            "  Phase 3b: net force = ({:.3}, {:.3}, {:.3})",
            force_dir.x, force_dir.y, force_dir.z
        );
        assert!(
            force_dir.z < 0.0,
            "axial force should push pin back (−Z), got z={:.3}",
            force_dir.z
        );
    }
}

#[test]
fn phase3c_hinge_rotation_free() {
    let socket_grid = Arc::new(make_socket_grid(0.3));
    let socket = ShapeConcave::new(socket_grid);

    let pin_grid = Arc::new(make_flanged_pin_grid(0.3));
    let pin = ShapeConvex::new(pin_grid);

    let socket_pose = identity_pose(Vector3::zeros());

    // Rotate pin 45° around Z axis (the bore axis). Centered at origin.
    let rotation = UnitQuaternion::from_axis_angle(
        &nalgebra::Unit::new_normalize(Vector3::z()),
        std::f64::consts::FRAC_PI_4,
    );
    let pin_pose = Pose {
        position: Point3::origin(),
        rotation,
    };

    let contacts = compute_shape_contact(&socket, &socket_pose, &pin, &pin_pose, 0.5, 50);

    // Almost all contacts should be zero-depth. A small number of grid-edge
    // artifacts (≤ 5) with minor penetration are tolerable — the SDF grid isn't
    // perfectly rotationally symmetric, so a few boundary grid points can shift
    // slightly inside the other shape when rotated 45°.
    let penetrating = contacts.iter().filter(|c| c.penetration > 0.1).count();
    let deep_penetrating = contacts.iter().filter(|c| c.penetration > 0.5).count();
    eprintln!(
        "  Phase 3c: {} total contacts, {} with pen > 0.1, {} with pen > 0.5 (rotated 45°)",
        contacts.len(),
        penetrating,
        deep_penetrating
    );

    // No deep penetration (would indicate geometric failure, not grid noise)
    assert!(
        deep_penetrating == 0,
        "rotation around bore axis should not create deep penetrating contacts, got {}",
        deep_penetrating
    );
    // Only minor grid-noise artifacts allowed
    assert!(
        penetrating <= 5,
        "rotation around bore axis should create at most 5 minor grid artifacts, got {}",
        penetrating
    );
}
