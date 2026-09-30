//! The conformance fixtures (recon §17b), each a model, an obstacle and one
//! state set on every executor, deformed 1–10 % unless named; and the margins
//! each asserts from the CPU's state: clear of where an output jumps.

#![cfg(test)]

use sim_soft_explicit::executor::{Obstacle, PhaseOutputs, Snapshot};
use sim_soft_explicit::f64::{Material, Pose, SdfGridLayout};
use sim_soft_explicit::fixtures::grid::bricks;
use sim_soft_explicit::fixtures::tube::{Insertion, Mandrel, Mesh, Tube, Walls};
use sim_soft_explicit::{ExplicitModel, f64 as shared};

/// Silicone-like: μ 23 kPa, ν 0.49, ρ 1070 kg/m³.
pub const SILICONE: Material = material(23.0e3, 0.49, 0.0);

/// A material of shear modulus `mu`, Poisson's ratio `nu` and viscosity
/// `viscosity`, at the silicone's density.
pub const fn material(mu: f64, nu: f64, viscosity: f64) -> Material {
    Material {
        mu,
        lambda: mu * 2.0 * nu / (1.0 - 2.0 * nu),
        c2: 0.0,
        viscosity,
        density: 1070.0,
    }
}

pub const IDENTITY: Pose = Pose {
    qw: 1.0,
    qx: 0.0,
    qy: 0.0,
    qz: 0.0,
    tx: 0.0,
    ty: 0.0,
    tz: 0.0,
};

/// A model, an obstacle, and the state and step every executor is given.
pub struct Fixture {
    pub name: &'static str,
    pub model: ExplicitModel,
    pub obstacle: Obstacle,
    pub time: f64,
    pub dt: f64,
    pub damping: f64,
    pub displacements: Vec<[f64; 3]>,
    pub velocities: Vec<[f64; 3]>,
    pub anchors: Option<Vec<[f64; 3]>>,
    /// Whether its model and step show what the fixture is for.
    pub shows: fn(&ExplicitModel, &Margins) -> bool,
}

/// Every fixture §17b names.
pub fn all() -> Vec<Fixture> {
    vec![
        tube(),
        two_materials(),
        viscous(),
        stabilized(),
        constrained(),
        fine_grid(),
        moving_and_turning(),
        sticking_and_slipping(),
        compressed(),
    ]
}

/// A block of `n.0 × n.1 × n.2` cubes of side `side`, each split into six
/// tetrahedra around one body diagonal, positively oriented.
pub fn block(n: (usize, usize, usize), side: f64) -> (Vec<[f64; 3]>, Vec<[u32; 4]>) {
    let node = |i: usize, j: usize, k: usize| ((k * (n.1 + 1) + j) * (n.0 + 1) + i) as u32;
    let mut positions = Vec::new();
    for k in 0..=n.2 {
        for j in 0..=n.1 {
            for i in 0..=n.0 {
                positions.push([i as f64 * side, j as f64 * side, k as f64 * side]);
            }
        }
    }
    let axes = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    let orders = [
        [0, 1, 2],
        [0, 2, 1],
        [1, 0, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0],
    ];
    let mut elements = Vec::new();
    for k in 0..n.2 {
        for j in 0..n.1 {
            for i in 0..n.0 {
                for order in orders {
                    let mut corner = [0, 0, 0];
                    let mut tet = [node(i, j, k); 4];
                    for (slot, &axis) in order.iter().enumerate() {
                        for d in 0..3 {
                            corner[d] += axes[axis][d];
                        }
                        tet[slot + 1] = node(i + corner[0], j + corner[1], k + corner[2]);
                    }
                    if shared::tet4_volume(gather(&positions, tet)) < 0.0 {
                        tet.swap(1, 2);
                    }
                    elements.push(tet);
                }
            }
        }
    }
    (positions, elements)
}

/// One element's nodal vectors, as the shared math takes them.
pub fn gather(values: &[[f64; 3]], element: [u32; 4]) -> [f64; 12] {
    let mut x = [0.0; 12];
    for (slot, &node) in element.iter().enumerate() {
        x[3 * slot..3 * slot + 3].copy_from_slice(&values[node as usize]);
    }
    x
}

/// A block of cubes of 10 mm in `material`, the nodes `held` says held.
pub fn block_model(
    n: (usize, usize, usize),
    material: impl Fn([f64; 3]) -> Material,
    held: impl Fn([f64; 3]) -> bool,
) -> ExplicitModel {
    let (positions, elements) = block(n, 0.01);
    let materials = elements
        .iter()
        .map(|&e| {
            let x = gather(&positions, e);
            material([
                (x[0] + x[3] + x[6] + x[9]) / 4.0,
                (x[1] + x[4] + x[7] + x[10]) / 4.0,
                (x[2] + x[5] + x[8] + x[11]) / 4.0,
            ])
        })
        .collect();
    let held = positions.iter().map(|&p| held(p)).collect();
    ExplicitModel::new(positions, elements, materials, held).unwrap()
}

/// A smooth, non-uniform deformation's displacements over a body about
/// `size` across, of strain about `amount`.
pub fn deformation(model: &ExplicitModel, size: f64, amount: f64) -> Vec<[f64; 3]> {
    let s = 3.0 / size;
    model
        .rest_positions()
        .iter()
        .map(|&[x, y, z]| {
            [
                amount * (0.3 * x + 0.2 * s * y * z + 0.05 / s * (7.0 * s * y).sin()),
                amount * (-0.25 * y + 0.15 * x + 0.04 / s * (5.0 * s * z).cos()),
                amount * (0.1 * z - 0.2 * s * x * y + 0.06 / s * (6.0 * s * x).sin()),
            ]
        })
        .collect()
}

/// A velocity field.
pub fn velocities(model: &ExplicitModel) -> Vec<[f64; 3]> {
    (0..model.node_count())
        .map(|a| [0.01 * (a as f64).sin(), -0.02, 0.005 * (a as f64).cos()])
        .collect()
}

/// A floor: in its body frame the surface is `z = 0` and the inside `z < 0`,
/// baked at `cell` over `[low, high]`, on the pose track `poses` sampled
/// every `interval` from time 0.
pub fn floor(
    (low, high, cell): ([f64; 3], [f64; 3], f64),
    interval: f64,
    poses: Vec<Pose>,
    friction: f64,
) -> Obstacle {
    let size = |axis: usize| ((high[axis] - low[axis]) / cell).round() as u32 + 1;
    let grid = SdfGridLayout {
        origin_x: low[0],
        origin_y: low[1],
        origin_z: low[2],
        cell_size: cell,
        size_x: size(0),
        size_y: size(1),
        size_z: size(2),
    };
    let mut values = Vec::new();
    for k in 0..grid.size_z {
        for _ in 0..grid.size_y * grid.size_x {
            values.push(low[2] + f64::from(k) * cell);
        }
    }
    Obstacle {
        grid,
        values,
        fine: None,
        start: 0.0,
        interval,
        poses,
        friction,
    }
}

/// A pose track moving by `velocity` per second from `start`, 101 samples
/// over 0.1 s.
pub fn sliding(start: [f64; 3], velocity: [f64; 3]) -> (f64, Vec<Pose>) {
    let interval = 0.001;
    let poses = (0..=100)
        .map(|i| {
            let t = f64::from(i) * interval;
            Pose {
                tx: start[0] + velocity[0] * t,
                ty: start[1] + velocity[1] * t,
                tz: start[2] + velocity[2] * t,
                ..IDENTITY
            }
        })
        .collect();
    (interval, poses)
}

/// A floor far below everything, never touched.
pub fn nowhere() -> Obstacle {
    floor(
        ([-1.0; 3], [1.0; 3], 1.0),
        1.0,
        vec![Pose {
            tz: -10.0,
            ..IDENTITY
        }],
        0.0,
    )
}

/// A floor under a block of cubes of 10 mm, 1 mm up into its bottom face at
/// time 0.075 and sliding along x.
fn rising_floor(friction: f64) -> Obstacle {
    let (interval, poses) = sliding([0.0, 0.0, -0.0005], [0.005, 0.0, 0.02]);
    floor(
        ([-0.01, -0.01, -0.005], [0.04, 0.04, 0.005], 0.001),
        interval,
        poses,
        friction,
    )
}

/// The time [`rising_floor`] is 1 mm up.
const RISEN: f64 = 0.075;

/// The tube on a mandrel 5 % wider than its bore, frictional, the nose 40 mm
/// in.
pub fn tube() -> Fixture {
    let tube = Tube::plan(Mesh::Cells {
        radial: 2,
        circumferential: 16,
        axial: 8,
    });
    let model = tube.model(SILICONE, Walls::Free).unwrap();
    let insertion = Insertion::plan(0.5);
    let mandrel = Mandrel {
        radius: 1.05 * tube.inner_radius,
    };
    let obstacle = insertion.obstacle(mandrel, &tube, 0.0005, 0.3).unwrap();
    // The time the nose is 40 mm in.
    let time = (0..=1000)
        .map(|i| f64::from(i) * 0.0005)
        .find(|&t| insertion.tip(t) >= 0.04)
        .unwrap();
    Fixture {
        name: "the tube, frictional",
        shows: |_, m| m.in_contact > 0,
        displacements: deformation(&model, tube.length, 0.01),
        velocities: velocities(&model),
        model,
        obstacle,
        time,
        dt: 1e-5,
        damping: 30.0,
        anchors: None,
    }
}

/// Two materials side by side, at ν 0.49 and 0.4975.
pub fn two_materials() -> Fixture {
    let model = block_model(
        (4, 3, 2),
        |c| {
            if c[0] < 0.02 {
                SILICONE
            } else {
                material(60.0e3, 0.4975, 0.0)
            }
        },
        |_| false,
    );
    Fixture {
        name: "two materials",
        shows: |model, _| {
            let ratio = |m: &Material| m.lambda / m.mu;
            let first = ratio(&model.materials()[0]);
            model.materials().iter().any(|m| ratio(m) != first)
        },
        displacements: deformation(&model, 0.04, 0.05),
        velocities: velocities(&model),
        model,
        obstacle: nowhere(),
        time: 0.0,
        dt: 1e-5,
        damping: 30.0,
        anchors: None,
    }
}

/// The silicone with its viscosity (7 Pa·s).
pub fn viscous() -> Fixture {
    let model = block_model((3, 3, 2), |_| material(23.0e3, 0.49, 7.0), |_| false);
    Fixture {
        name: "viscous",
        shows: |model, _| model.materials().iter().all(|m| m.viscosity > 0.0),
        displacements: deformation(&model, 0.03, 0.05),
        velocities: velocities(&model),
        model,
        obstacle: nowhere(),
        time: 0.0,
        dt: 1e-5,
        damping: 0.0,
        anchors: None,
    }
}

/// A block with a volumetric stabilization of twice μ (recon §16y), which
/// the product runs without.
pub fn stabilized() -> Fixture {
    let model = block_model((3, 2, 2), |_| SILICONE, |_| false)
        .with_volumetric_stabilization(2.0)
        .unwrap();
    Fixture {
        name: "a volumetric stabilization",
        shows: |model, _| model.element_stabilizations().iter().all(|&k| k > 0.0),
        displacements: deformation(&model, 0.03, 0.05),
        velocities: velocities(&model),
        model,
        obstacle: nowhere(),
        time: 0.0,
        dt: 1e-5,
        damping: 30.0,
        anchors: None,
    }
}

/// A block pressed on a frictional floor, its top held whole, the face `x = 0`
/// constrained along x and the face `y = 0` along y and z; so the floor's
/// normal lies in some nodes' constrained directions.
pub fn constrained() -> Fixture {
    let model = block_model((3, 3, 2), |_| SILICONE, |p| p[2] > 0.02 - 1e-9);
    let constraints = model
        .rest_positions()
        .iter()
        .map(|p| {
            if p[1] < 1e-9 {
                [[0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
            } else if p[0] < 1e-9 {
                [[1.0, 0.0, 0.0], [0.0; 3]]
            } else {
                [[0.0; 3]; 2]
            }
        })
        .collect();
    let model = model.with_constraints(constraints).unwrap();
    Fixture {
        name: "held and constrained",
        shows: |_, m| m.constrained_contacts > 0,
        displacements: deformation(&model, 0.03, 0.03),
        velocities: velocities(&model),
        model,
        obstacle: rising_floor(0.3),
        time: RISEN,
        dt: 1e-5,
        damping: 30.0,
        anchors: None,
    }
}

/// A block pressed on a floor with a fine grid under part of it, 0.1 mm
/// lower than the coarse grid's, so a depth read from the wrong grid shows.
fn fine_grid() -> Fixture {
    let model = block_model((3, 3, 2), |_| SILICONE, |p| p[2] > 0.02 - 1e-9);
    let coarse = rising_floor(0.0);
    let mut fine = floor(
        ([-0.01, -0.01, -0.005], [0.04, 0.04, 0.005], 0.0005),
        1.0,
        vec![IDENTITY],
        0.0,
    );
    for value in &mut fine.values {
        *value += 1e-4;
    }
    let obstacle = Obstacle {
        fine: Some(bricks(fine.grid, &fine.values, |[i, _, _]| i < 3)),
        ..coarse
    };
    Fixture {
        name: "a fine grid",
        shows: |_, m| m.fine_contacts > 0 && m.fine_contacts < m.in_contact,
        displacements: deformation(&model, 0.03, 0.03),
        velocities: velocities(&model),
        model,
        obstacle,
        time: RISEN,
        dt: 1e-5,
        damping: 30.0,
        anchors: None,
    }
}

/// A floor tilted 0.1 rad about x, turning about y at 1 rad/s and rising,
/// pressed into a block's bottom face, frictional.
pub fn moving_and_turning() -> Fixture {
    let model = block_model((3, 3, 2), |_| SILICONE, |p| p[2] > 0.02 - 1e-9);
    Fixture {
        name: "an obstacle that moves and turns",
        shows: |_, m| m.in_contact > 0,
        displacements: deformation(&model, 0.03, 0.03),
        velocities: velocities(&model),
        model,
        obstacle: turning_floor(1.0),
        time: 0.09,
        dt: 1e-5,
        damping: 30.0,
        anchors: None,
    }
}

/// A floor tilted 0.1 rad about x and turning about y at `rate`, its origin
/// under a block of cubes of 10 mm and rising, frictional.
pub fn turning_floor(rate: f64) -> Obstacle {
    let samples = 101_u32;
    let interval = 0.1 / f64::from(samples - 1);
    let (tilt, centre) = (0.1_f64, 0.015);
    let poses = (0..samples)
        .map(|i| {
            let t = f64::from(i) * interval;
            let (c1, s1) = ((0.5 * rate * t).cos(), (0.5 * rate * t).sin());
            let (c2, s2) = ((0.5 * tilt).cos(), (0.5 * tilt).sin());
            // The turn about y after the tilt about x: q_y ⊗ q_x.
            Pose {
                qw: c1 * c2,
                qx: c1 * s2,
                qy: s1 * c2,
                qz: -s1 * s2,
                tx: centre + 0.005 * t,
                ty: centre,
                tz: -0.002 + 0.05 * t,
            }
        })
        .collect();
    floor(
        ([-0.04, -0.04, -0.04], [0.04, 0.04, 0.04], 0.002),
        interval,
        poses,
        0.3,
    )
}

/// A block pressed on a frictional floor, its bottom nodes anchored so some
/// stick and some slip: anchors 0.02 mm and 1 mm from where each sits, the
/// friction limit 0.3 of the press.
fn sticking_and_slipping() -> Fixture {
    let model = block_model((3, 3, 2), |_| SILICONE, |p| p[2] > 0.02 - 1e-9);
    let displacements = deformation(&model, 0.03, 0.03);
    let obstacle = rising_floor(0.3);
    let pose = obstacle.pose_at(RISEN);
    let anchors = model
        .rest_positions()
        .iter()
        .zip(&displacements)
        .enumerate()
        .map(|(a, (rest, u))| {
            let at = shared::pose_to_body(pose, shared::vec3_add(*rest, *u));
            let offset = if a % 2 == 0 { 2e-5 } else { 1e-3 };
            let angle = a as f64;
            shared::vec3_add(at, [offset * angle.cos(), offset * angle.sin(), 0.0])
        })
        .collect();
    Fixture {
        name: "friction sticking and slipping",
        shows: |_, m| m.slipping > 0 && m.slipping < m.in_contact,
        displacements,
        velocities: velocities(&model),
        model,
        obstacle,
        time: RISEN,
        dt: 1e-5,
        damping: 30.0,
        anchors: Some(anchors),
    }
}

/// A block compressed 30 % along z, past `|J − 1| = 0.25`, where `ln_1p`
/// takes the backend's `log`.
fn compressed() -> Fixture {
    let model = block_model((3, 2, 2), |_| SILICONE, |_| false);
    let displacements = deformation(&model, 0.03, 0.02)
        .iter()
        .zip(model.rest_positions())
        .map(|(u, p)| [u[0], u[1], u[2] - 0.3 * p[2]])
        .collect();
    Fixture {
        name: "compressed past |J - 1| = 0.25",
        shows: |_, m| m.jacobian < 0.75,
        displacements,
        velocities: velocities(&model),
        model,
        obstacle: nowhere(),
        time: 0.0,
        dt: 1e-5,
        damping: 30.0,
        anchors: None,
    }
}

/// How close the fixture's step comes to where an output jumps, read from
/// the CPU's state before the step and its phase outputs.
#[derive(Debug)]
pub struct Margins {
    /// The nearest a movable surface node's predicted depth comes to 0.
    pub touch: f64,
    /// The nearest a node in contact comes to turning between sticking and
    /// slipping: `|tangential slip − friction limit|`; infinite without
    /// friction.
    pub friction: f64,
    /// The nearest a movable node's reach comes to `KINEMATIC_MIN_REACH`.
    pub reach: f64,
    /// Whether some lookup's grid, fine or coarse, changes within 1 µm.
    pub fine_edge: bool,
    /// The nearest an element comes to `J = 0`.
    pub jacobian: f64,
    /// Surface nodes in contact.
    pub in_contact: usize,
    /// Of them, slipping.
    pub slipping: usize,
    /// Of them, answered by the fine grid.
    pub fine_contacts: usize,
    /// Surface nodes inside the obstacle with its normal partly or wholly in
    /// a constrained direction.
    pub constrained_contacts: usize,
}

impl Margins {
    /// Assert every margin: 1 µm of depth, 0.1 µm of slip, 1e-4 of reach, no
    /// grid change within 1 µm, and 0.1 of `J` (at `J` = 0.002 the pressures
    /// at f32 missed their bar).
    pub fn assert_clear(&self, name: &str) {
        assert!(
            self.touch > 1e-6,
            "{name}: a node {:e} m from touching",
            self.touch
        );
        assert!(
            self.friction > 1e-7,
            "{name}: a node {:e} m from turning to slip",
            self.friction
        );
        assert!(
            self.reach > 1e-4,
            "{name}: a node's reach {:e} from the least",
            self.reach
        );
        assert!(!self.fine_edge, "{name}: a lookup at the fine grid's edge");
        assert!(
            self.jacobian > 0.1,
            "{name}: an element {:e} from J = 0",
            self.jacobian
        );
    }
}

/// Whether the grid that answers a lookup at `point`, fine or coarse, changes
/// within 1 µm of it.
fn fine_changes(obstacle: &Obstacle, point: [f64; 3]) -> bool {
    obstacle.fine.as_ref().is_some_and(|fine| {
        let here = fine.sample(point).is_some();
        (0..3).any(|axis| {
            [-1e-6, 1e-6].iter().any(|&step| {
                let mut moved = point;
                moved[axis] += step;
                fine.sample(moved).is_some() != here
            })
        })
    })
}

/// The margins of `fixture`'s step, from the state `before` it and the
/// phase outputs of it, at f64 on the CPU's values.
pub fn margins(fixture: &Fixture, before: &Snapshot, outputs: &PhaseOutputs) -> Margins {
    let (model, obstacle) = (&fixture.model, &fixture.obstacle);
    let (dt, damping) = (fixture.dt, fixture.damping);
    let (pose, next) = (
        obstacle.pose_at(fixture.time),
        obstacle.pose_at(fixture.time + dt),
    );
    let surface = model.surface_incidence();
    let mut margins = Margins {
        touch: f64::INFINITY,
        friction: f64::INFINITY,
        reach: f64::INFINITY,
        fine_edge: false,
        jacobian: outputs
            .dilations
            .iter()
            .fold(f64::INFINITY, |m, &d| m.min(1.0 + d)),
        in_contact: 0,
        slipping: 0,
        fine_contacts: 0,
        constrained_contacts: 0,
    };
    for a in 0..model.node_count() {
        if surface.of(a).is_empty() {
            continue;
        }
        let (mass, held) = (model.node_masses()[a], model.held()[a]);
        let inverse_mass = shared::inverse_mass(mass, held);
        let [first, second] = model.constraints()[a];
        let free = |v: [f64; 3]| {
            if inverse_mass == 0.0 {
                [0.0; 3]
            } else {
                shared::constrain(v, first, second)
            }
        };
        let rest = model.rest_positions()[a];
        let (u, v) = (before.displacements[a], before.velocities[a]);
        let now = shared::pose_to_body(pose, shared::vec3_add(rest, u));
        let force = shared::vec3_add(outputs.elastic_forces[a], outputs.viscous_forces[a]);
        let velocity = free(shared::advance_velocity(
            v,
            force,
            inverse_mass,
            damping,
            dt,
        ));
        let point = shared::vec3_add(rest, free(shared::advance_displacement(u, velocity, dt)));
        let predicted = shared::pose_to_body(next, point);
        margins.fine_edge |= fine_changes(obstacle, now) || fine_changes(obstacle, predicted);
        let stiffness = shared::kinematic_stiffness(mass, inverse_mass, damping, dt);
        if stiffness == 0.0 {
            continue;
        }
        let depth = obstacle.sample(predicted).distance;
        margins.touch = margins.touch.min(depth.abs());
        let body_normal =
            shared::pose_unrotate(next, shared::pose_rotate(pose, obstacle.sample(now).normal));
        let normal = shared::pose_rotate(next, body_normal);
        let along = shared::constrain(normal, first, second);
        let reach = shared::vec3_dot(along, normal);
        margins.reach = margins
            .reach
            .min((reach - shared::KINEMATIC_MIN_REACH).abs());
        if depth < 0.0 && reach < 1.0 - 1e-9 {
            margins.constrained_contacts += 1;
        }
        if reach <= shared::KINEMATIC_MIN_REACH || depth >= 0.0 {
            continue;
        }
        margins.in_contact += 1;
        if obstacle
            .fine
            .as_ref()
            .is_some_and(|fine| fine.sample(predicted).is_some())
        {
            margins.fine_contacts += 1;
        }
        let penetration = -depth;
        let corrected = shared::vec3_add(point, shared::vec3_scale(along, penetration / reach));
        let anchor = fixture.anchors.as_ref().map_or(now, |anchors| anchors[a]);
        let slip = shared::vec3_sub(shared::pose_to_body(next, corrected), anchor);
        let tangential = shared::vec3_sub(
            slip,
            shared::vec3_scale(body_normal, shared::vec3_dot(slip, body_normal)),
        );
        let limit = obstacle.friction * penetration / reach;
        let length = shared::vec3_length(tangential);
        if length > limit {
            margins.slipping += 1;
        }
        if obstacle.friction > 0.0 {
            margins.friction = margins.friction.min((length - limit).abs());
        }
    }
    margins
}
