// The CPU executor at one precision. `src/cpu.rs` includes this file twice,
// with `R` as `f32` and as `f64` and `shared` as the matching shared math, so
// the two precisions run the same code and K3 compares them (plan §15a).
//
// Per-element and per-node work goes through `fill` and `update`, which run
// under rayon on native and in order on wasm32. A gather sums each node's
// incidence list in its fixed order, so no two threads add into one place
// and the result does not depend on how the work is split. Reductions for
// the monitors run in node order, in f64.

/// An obstacle sample before the first contact phase.
const NO_SAMPLE: shared::SdfSample = shared::SdfSample {
    distance: 0.0,
    normal: [0.0; 3],
};

/// A surface node's position at the end of a step without contact, the
/// obstacle's distance there, and whether the fine grid answered.
#[derive(Clone, Copy, Debug)]
struct Prediction {
    point: [R; 3],
    depth: R,
    fine: bool,
}

/// A prediction before the first contact phase.
const NO_PREDICTION: Prediction = Prediction {
    point: [0.0; 3],
    depth: 0.0,
    fine: true,
};

/// A boundary value at this executor's precision.
// f64 → f32 rounds, which is the point; at f64 it is the identity.
#[allow(clippy::cast_possible_truncation, clippy::unnecessary_cast)]
const fn narrow(x: f64) -> R {
    x as R
}

/// A value at this executor's precision, widened for the boundary.
// At f64 the conversion is the identity; at f32 it widens.
#[allow(clippy::useless_conversion)]
fn widen(x: R) -> f64 {
    f64::from(x)
}

const fn narrow3(v: [f64; 3]) -> [R; 3] {
    [narrow(v[0]), narrow(v[1]), narrow(v[2])]
}

fn widen3(v: [R; 3]) -> [f64; 3] {
    [widen(v[0]), widen(v[1]), widen(v[2])]
}

/// Each node's `λ_a − κ_a`, the λ its averaged term uses (plan §16y), formed
/// at `f64` and then narrowed.
fn averaged_lambdas(model: &ExplicitModel) -> Vec<R> {
    model
        .node_lambdas()
        .iter()
        .zip(model.node_stabilizations())
        .map(|(&lambda, &stabilization)| narrow(lambda - stabilization))
        .collect()
}
/// A grid layout at this executor's precision.
const fn narrow_layout(grid: crate::f64::SdfGridLayout) -> shared::SdfGridLayout {
    shared::SdfGridLayout {
        origin_x: narrow(grid.origin_x),
        origin_y: narrow(grid.origin_y),
        origin_z: narrow(grid.origin_z),
        cell_size: narrow(grid.cell_size),
        size_x: grid.size_x,
        size_y: grid.size_y,
        size_z: grid.size_z,
    }
}

const fn narrow_material(m: crate::f64::Material) -> shared::Material {
    shared::Material {
        mu: narrow(m.mu),
        lambda: narrow(m.lambda),
        c2: narrow(m.c2),
        viscosity: narrow(m.viscosity),
        density: narrow(m.density),
    }
}

const fn narrow_pose(p: crate::f64::Pose) -> shared::Pose {
    shared::Pose {
        qw: narrow(p.qw),
        qx: narrow(p.qx),
        qy: narrow(p.qy),
        qz: narrow(p.qz),
        tx: narrow(p.tx),
        ty: narrow(p.ty),
        tz: narrow(p.tz),
    }
}

/// One element's nodal vectors, as the shared math takes them.
fn gather12(values: &[[R; 3]], element: [u32; 4]) -> [R; 12] {
    let [a, b, c, d] = element.map(|n| values[n as usize]);
    [
        a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2],
    ]
}

/// A node's slots in an incidence list.
fn slots<'a>(offsets: &[u32], entries: &'a [u32], node: usize) -> &'a [u32] {
    &entries[offsets[node] as usize..offsets[node + 1] as usize]
}

/// The per-element arrays phases 1–5 read, borrowed together so the force
/// evaluation can run on any displacement field.
struct Elastic<'a> {
    elements: &'a [[u32; 4]],
    materials: &'a [shared::Material],
    rest_edge_inverses: &'a [[R; 9]],
    rest_volumes: &'a [R],
    node_rest_volumes: &'a [R],
    averaged_lambdas: &'a [R],
    element_stabilizations: &'a [R],
    offsets: &'a [u32],
    entries: &'a [u32],
}

impl Elastic<'_> {
    /// Phase 1 into `dilations`.
    fn dilations(&self, displacements: &[[R; 3]], dilations: &mut [R]) {
        fill(dilations, |e| {
            shared::tet4_dilation(
                gather12(displacements, self.elements[e]),
                self.rest_edge_inverses[e],
            )
        });
    }

    /// Phase 2 into `volume_changes`: `Δv_a = Σ V_e (J_e − 1) / 4`.
    fn volume_changes(&self, dilations: &[R], volume_changes: &mut [R]) {
        fill(volume_changes, |a| {
            slots(self.offsets, self.entries, a)
                .iter()
                .fold(0.0, |sum, &slot| {
                    let e = slot as usize / 4;
                    sum + 0.25 * self.rest_volumes[e] * dilations[e]
                })
        });
    }

    /// Phase 3 into `pressures`.
    fn pressures(&self, volume_changes: &[R], pressures: &mut [R]) {
        fill(pressures, |a| {
            shared::pressure_lambda_term(
                shared::nodal_dilation(volume_changes[a], self.node_rest_volumes[a]),
                self.averaged_lambdas[a],
            )
        });
    }

    /// Phase 4 into `element_forces`, from phase 1's `dilations` and phase
    /// 3's `pressures`.
    fn element_forces(
        &self,
        displacements: &[[R; 3]],
        dilations: &[R],
        pressures: &[R],
        element_forces: &mut [[R; 12]],
    ) {
        fill(element_forces, |e| {
            let [a, b, c, d] = self.elements[e].map(|n| pressures[n as usize]);
            shared::tet4_elastic_forces(
                gather12(displacements, self.elements[e]),
                self.rest_edge_inverses[e],
                self.rest_volumes[e],
                self.materials[e],
                shared::sampled_element_pressure(
                    [a, b, c, d],
                    dilations[e],
                    self.element_stabilizations[e],
                ),
            )
        });
    }

    /// Phase 5 into `forces`.
    fn gather_forces(&self, element_forces: &[[R; 12]], forces: &mut [[R; 3]]) {
        fill(forces, |a| {
            slots(self.offsets, self.entries, a)
                .iter()
                .fold([0.0; 3], |sum, &slot| {
                    let (e, corner) = (slot as usize / 4, slot as usize % 4);
                    let f = &element_forces[e][3 * corner..3 * corner + 3];
                    [sum[0] + f[0], sum[1] + f[1], sum[2] + f[2]]
                })
        });
    }

    /// Phase 4's viscous part into `element_viscous_forces`, at `velocities`.
    fn viscous_forces(
        &self,
        displacements: &[[R; 3]],
        velocities: &[[R; 3]],
        element_viscous_forces: &mut [[R; 12]],
    ) {
        fill(element_viscous_forces, |e| {
            shared::tet4_viscous_forces(
                gather12(displacements, self.elements[e]),
                gather12(velocities, self.elements[e]),
                self.rest_edge_inverses[e],
                self.rest_volumes[e],
                self.materials[e],
            )
        });
    }

    /// Phases 1–5 on `displacements`, into fresh arrays.
    fn forces(&self, displacements: &[[R; 3]]) -> Vec<[R; 3]> {
        let nodes = self.node_rest_volumes.len();
        let mut dilations = vec![0.0; self.elements.len()];
        let mut volume_changes = vec![0.0; nodes];
        let mut pressures = vec![0.0; nodes];
        let mut element_forces = vec![[0.0; 12]; self.elements.len()];
        let mut forces = vec![[0.0; 3]; nodes];
        self.dilations(displacements, &mut dilations);
        self.volume_changes(&dilations, &mut volume_changes);
        self.pressures(&volume_changes, &mut pressures);
        self.element_forces(displacements, &dilations, &pressures, &mut element_forces);
        self.gather_forces(&element_forces, &mut forces);
        forces
    }

    /// The internal energy of `displacements`: the μ terms and the sampled
    /// part of the λ term per element, and the rest of the λ term averaged
    /// per node, `Σ_e V_e (Ψ_μ + κ_e/2 (ln J_e)²) + Σ_a V_a (λ_a − κ_a)/2
    /// (ln J_a)²`.
    fn energy(&self, displacements: &[[R; 3]]) -> f64 {
        let nodes = self.node_rest_volumes.len();
        let mut dilations = vec![0.0; self.elements.len()];
        let mut volume_changes = vec![0.0; nodes];
        self.dilations(displacements, &mut dilations);
        self.volume_changes(&dilations, &mut volume_changes);
        let elements: f64 = (0..self.elements.len())
            .map(|e| {
                widen(
                    shared::tet4_energy_mu_terms(
                        gather12(displacements, self.elements[e]),
                        self.rest_edge_inverses[e],
                        self.rest_volumes[e],
                        self.materials[e],
                    ) + self.rest_volumes[e]
                        * shared::energy_density_lambda_term(
                            dilations[e],
                            self.element_stabilizations[e],
                        ),
                )
            })
            .sum();
        let nodes: f64 = (0..nodes)
            .map(|a| {
                widen(
                    self.node_rest_volumes[a]
                        * shared::energy_density_lambda_term(
                            shared::nodal_dilation(volume_changes[a], self.node_rest_volumes[a]),
                            self.averaged_lambdas[a],
                        ),
                )
            })
            .sum();
        elements + nodes
    }
}

/// The explicit solver's state on the CPU, at this module's precision.
///
/// Built from a lowered [`ExplicitModel`] and an [`Obstacle`]; stepped by
/// [`crate::stepping::Stepper`] through the [`Executor`] phases.
#[derive(Clone, Debug)]
pub struct CpuExecutor {
    rest: Vec<[R; 3]>,
    elements: Vec<[u32; 4]>,
    materials: Vec<shared::Material>,
    rest_edge_inverses: Vec<[R; 9]>,
    rest_volumes: Vec<R>,
    masses: Vec<R>,
    inverse_masses: Vec<R>,
    node_rest_volumes: Vec<R>,
    /// Each node's `λ_a − κ_a` ([`averaged_lambdas`]).
    averaged_lambdas: Vec<R>,
    element_stabilizations: Vec<R>,
    constraints: Vec<[[R; 3]; 2]>,
    offsets: Vec<u32>,
    entries: Vec<u32>,
    shortest_edge: f64,
    /// Whether any material has a viscosity. Without one the viscous forces
    /// are zero, and the phases and the step's estimate skip them.
    viscous: bool,

    /// The nodes on the boundary surface, the only ones that can touch the
    /// obstacle; the contact arrays are indexed by position in this list.
    surface_nodes: Vec<u32>,
    /// For each node, its position in `surface_nodes`, or `u32::MAX`.
    surface_index: Vec<u32>,

    grid: shared::SdfGridLayout,
    grid_values: Vec<R>,
    /// The obstacle's fine grid near its surface, if it has one: its layout,
    /// its brick map and its bricks' values.
    fine: Option<(shared::SdfGridLayout, Vec<u32>, Vec<R>)>,
    pose_start: R,
    pose_interval: R,
    poses: Vec<shared::Pose>,
    /// The pose track as given, at f64: the obstacle's move over a step, for
    /// its work, is read from it at both precisions.
    track: Track,
    friction: R,

    displacements: Vec<[R; 3]>,
    velocities: Vec<[R; 3]>,
    previous_velocities: Vec<[R; 3]>,
    dilations: Vec<R>,
    volume_changes: Vec<R>,
    pressures: Vec<R>,
    element_forces: Vec<[R; 12]>,
    elastic_forces: Vec<[R; 3]>,
    element_viscous_forces: Vec<[R; 12]>,
    viscous_forces: Vec<[R; 3]>,
    contacts: Vec<shared::ContactResponse>,
    anchors: Vec<[R; 3]>,
    stiffnesses: Vec<R>,
    /// Each surface node's obstacle sample where it sits this step: the
    /// normal its correction follows, and G2's depth.
    samples: Vec<shared::SdfSample>,
    /// Each surface node's position at the end of this step without
    /// contact, and the obstacle's distance there: the depth its correction
    /// takes.
    predictions: Vec<Prediction>,

    max_penetrations: Vec<R>,
    max_predictions: Vec<R>,
    coarse_corrections: u64,
    contact_work: Vec<f64>,
    damping_losses: Vec<f64>,
    inverted_element_steps: u64,
    displacement_sums: Vec<[f64; 3]>,
    normal_force_sums: Vec<f64>,
    friction_sums: Vec<[f64; 3]>,
    accumulated_steps: u64,
    since_read: SinceRead,
    obstacle_work: f64,
}

/// The monitors' sums over the steps since the last read, in f64.
#[derive(Clone, Copy, Debug, Default)]
struct SinceRead {
    resultant: [f64; 3],
    moment: [f64; 3],
    normal: f64,
    steps: u64,
}

/// A pose track at f64: samples every `interval` from `start`.
#[derive(Clone, Debug)]
struct Track {
    start: f64,
    interval: f64,
    poses: Vec<crate::f64::Pose>,
}

impl Track {
    /// The track of `poses`, samples every `interval` from `start`.
    fn of(start: f64, interval: f64, poses: &[crate::f64::Pose]) -> Self {
        Self {
            start,
            interval,
            poses: poses.to_vec(),
        }
    }

    /// The pose at `time`, interpolated as the executor interpolates it.
    // `poses.len()` fits a u32: a pose track is a few thousand samples.
    #[allow(clippy::cast_possible_truncation)]
    fn at(&self, time: f64) -> crate::f64::Pose {
        let span = crate::f64::pose_sample_span(
            time,
            self.start,
            self.interval,
            self.poses.len() as u32,
        );
        crate::f64::pose_interpolate(
            self.poses[span.lower as usize],
            self.poses[span.upper as usize],
            span.fraction,
        )
    }
}

impl CpuExecutor {
    /// An executor for `model` against `obstacle`, at rest.
    ///
    /// # Errors
    /// An [`ObstacleError`] naming the first problem with `obstacle`.
    pub fn new(model: &ExplicitModel, obstacle: &Obstacle) -> Result<Self, ObstacleError> {
        check_obstacle(obstacle)?;
        let nodes = model.node_count();
        let surface = model.surface_incidence();
        // Every node is in some element, and elements index nodes by `u32`, so
        // node indices fit; zipping with a `u32` counter keeps them exact.
        let surface_nodes: Vec<u32> = (0_u32..)
            .zip(0..nodes)
            .filter(|&(_, a)| !surface.of(a).is_empty())
            .map(|(index, _)| index)
            .collect();
        let mut surface_index = vec![u32::MAX; nodes];
        for (i, &a) in (0_u32..).zip(&surface_nodes) {
            surface_index[a as usize] = i;
        }
        let incidence = model.element_incidence();
        let grid = obstacle.grid;
        let empty_contact = shared::ContactResponse {
            force: [0.0; 3],
            anchor: [0.0; 3],
            normal_force: 0.0,
            friction: [0.0; 3],
        };
        let rest: Vec<[R; 3]> = model.rest_positions().iter().map(|&p| narrow3(p)).collect();
        // Each surface node starts anchored where it sits against the track's
        // first pose, in the obstacle's body frame, as the contact law reads
        // anchors: a node that starts in contact starts sticking there.
        let first = narrow_pose(obstacle.poses[0]);
        let anchors = surface_nodes
            .iter()
            .map(|&a| shared::pose_to_body(first, rest[a as usize]))
            .collect();
        let surface_count = surface_nodes.len();
        Ok(Self {
            elements: model.elements().to_vec(),
            materials: model.materials().iter().map(|&m| narrow_material(m)).collect(),
            rest_edge_inverses: model
                .rest_edge_inverses()
                .iter()
                .map(|m| m.map(narrow))
                .collect(),
            rest_volumes: model.rest_volumes().iter().map(|&v| narrow(v)).collect(),
            masses: model.node_masses().iter().map(|&m| narrow(m)).collect(),
            inverse_masses: model
                .node_masses()
                .iter()
                .zip(model.held())
                .map(|(&m, &held)| shared::inverse_mass(narrow(m), held))
                .collect(),
            node_rest_volumes: model.node_rest_volumes().iter().map(|&v| narrow(v)).collect(),
            averaged_lambdas: averaged_lambdas(model),
            element_stabilizations: model.element_stabilizations().iter().map(|&k| narrow(k)).collect(),
            constraints: model
                .constraints()
                .iter()
                .map(|[a, b]| [narrow3(*a), narrow3(*b)])
                .collect(),
            offsets: incidence.offsets().to_vec(),
            entries: incidence.entries().to_vec(),
            shortest_edge: model.shortest_edge(),
            viscous: model.materials().iter().any(|m| m.viscosity > 0.0),
            surface_nodes,
            surface_index,
            grid: narrow_layout(grid),
            grid_values: obstacle.values.iter().map(|&v| narrow(v)).collect(),
            fine: obstacle.fine.as_ref().map(|fine| {
                (
                    narrow_layout(fine.grid),
                    fine.map.clone(),
                    fine.values.iter().map(|&v| narrow(v)).collect(),
                )
            }),
            pose_start: narrow(obstacle.start),
            pose_interval: narrow(obstacle.interval),
            poses: obstacle.poses.iter().map(|&p| narrow_pose(p)).collect(),
            track: Track::of(obstacle.start, obstacle.interval, &obstacle.poses),
            friction: narrow(obstacle.friction),
            displacements: vec![[0.0; 3]; nodes],
            velocities: vec![[0.0; 3]; nodes],
            previous_velocities: vec![[0.0; 3]; nodes],
            dilations: vec![0.0; model.element_count()],
            volume_changes: vec![0.0; nodes],
            pressures: vec![0.0; nodes],
            element_forces: vec![[0.0; 12]; model.element_count()],
            elastic_forces: vec![[0.0; 3]; nodes],
            element_viscous_forces: vec![[0.0; 12]; model.element_count()],
            viscous_forces: vec![[0.0; 3]; nodes],
            contacts: vec![empty_contact; surface_count],
            anchors,
            stiffnesses: vec![0.0; surface_count],
            samples: vec![NO_SAMPLE; surface_count],
            predictions: vec![NO_PREDICTION; surface_count],
            max_penetrations: vec![0.0; surface_count],
            max_predictions: vec![0.0; surface_count],
            coarse_corrections: 0,
            contact_work: vec![0.0; nodes],
            damping_losses: vec![0.0; nodes],
            inverted_element_steps: 0,
            displacement_sums: vec![[0.0; 3]; nodes],
            normal_force_sums: vec![0.0; surface_count],
            friction_sums: vec![[0.0; 3]; surface_count],
            accumulated_steps: 0,
            since_read: SinceRead::default(),
            obstacle_work: 0.0,
            rest,
        })
    }

    fn elastic(&self) -> Elastic<'_> {
        Elastic {
            elements: &self.elements,
            materials: &self.materials,
            rest_edge_inverses: &self.rest_edge_inverses,
            rest_volumes: &self.rest_volumes,
            node_rest_volumes: &self.node_rest_volumes,
            averaged_lambdas: &self.averaged_lambdas,
            element_stabilizations: &self.element_stabilizations,
            offsets: &self.offsets,
            entries: &self.entries,
        }
    }

    /// The obstacle's pose at `time`, interpolated between its samples.
    // `poses.len()` fits a u32: a pose track is a few thousand samples.
    #[allow(clippy::cast_possible_truncation)]
    fn pose_at(&self, time: f64) -> shared::Pose {
        let span = shared::pose_sample_span(
            narrow(time),
            self.pose_start,
            self.pose_interval,
            self.poses.len() as u32,
        );
        shared::pose_interpolate(
            self.poses[span.lower as usize],
            self.poses[span.upper as usize],
            span.fraction,
        )
    }

    /// A node's internal force this step: elastic plus viscous.
    fn internal_force(&self, node: usize) -> [R; 3] {
        shared::vec3_add(self.elastic_forces[node], self.viscous_forces[node])
    }

    /// A node's contact force this step: zero off the surface.
    fn contact_force(&self, node: usize) -> [R; 3] {
        let i = self.surface_index[node];
        if i == u32::MAX {
            [0.0; 3]
        } else {
            self.contacts[i as usize].force
        }
    }

    /// Remove a node's held and constrained directions from `v`.
    fn free_part(&self, node: usize, v: [R; 3]) -> [R; 3] {
        if self.inverse_masses[node] == 0.0 {
            [0.0; 3]
        } else {
            let [first, second] = self.constraints[node];
            shared::constrain(v, first, second)
        }
    }

    /// The obstacle's distance and normal at a body-frame point: the shared
    /// tricubic lookup over the 64 grid values it names, in the fine grid
    /// where that has them all.
    fn sample(&self, point: [R; 3]) -> shared::SdfSample {
        self.sample_and_grid(point).0
    }

    /// [`Self::sample`], and whether the fine grid answered it.
    fn sample_and_grid(&self, point: [R; 3]) -> (shared::SdfSample, bool) {
        if let Some(sample) = self.fine_sample(point) {
            return (sample, true);
        }
        let grid = self.grid;
        let coordinate = shared::sdf_grid_coordinate(point, grid);
        let columns = shared::sdf_tricubic_axis(coordinate[0], grid.size_x);
        let rows = shared::sdf_tricubic_axis(coordinate[1], grid.size_y);
        let layers = shared::sdf_tricubic_axis(coordinate[2], grid.size_z);
        let mut values = [0.0; 64];
        for (k, &layer) in layers.iter().enumerate() {
            for (j, &row) in rows.iter().enumerate() {
                let start = shared::sdf_grid_index(columns[0], row, layer, grid) as usize;
                let line = &self.grid_values[start..];
                for (i, &column) in columns.iter().enumerate() {
                    values[(k * 4 + j) * 4 + i] = line[(column - columns[0]) as usize];
                }
            }
        }
        (shared::sdf_tricubic(coordinate, values, grid), false)
    }

    /// The lookup in the fine grid, or `None` where the point is off its
    /// lattice or one of its 64 samples has no brick (`Obstacle::sample`'s
    /// rule).
    fn fine_sample(&self, point: [R; 3]) -> Option<shared::SdfSample> {
        let (grid, map, bricks) = self.fine.as_ref()?;
        let grid = *grid;
        if !shared::sdf_on_grid(point, grid) {
            return None;
        }
        let coordinate = shared::sdf_grid_coordinate(point, grid);
        let columns = shared::sdf_tricubic_axis(coordinate[0], grid.size_x);
        let rows = shared::sdf_tricubic_axis(coordinate[1], grid.size_y);
        let layers = shared::sdf_tricubic_axis(coordinate[2], grid.size_z);
        let slots =
            shared::sdf_stencil_bricks(columns, rows, layers, grid).map(|entry| map[entry as usize]);
        if !shared::sdf_fine_present(slots) {
            return None;
        }
        let mut values = [0.0; 64];
        for (index, value) in values.iter_mut().enumerate() {
            let (column, row, layer) = (columns[index % 4], rows[index / 4 % 4], layers[index / 16]);
            *value = bricks[shared::sdf_fine_index(
                column, row, layer, columns[0], rows[0], layers[0], slots,
            ) as usize];
        }
        Some(shared::sdf_tricubic(coordinate, values, grid))
    }

    /// The power iteration of [`Executor::estimate_top_mode`], and the
    /// vector its quotients were taken of: where the mode that sets the step
    /// sits (plan §16y). A diagnostic; the stepping loop never reads it.
    // Node indices feed a deterministic starting vector; precision loss in
    // `usize → R` only changes which starting vector it is.
    #[allow(clippy::cast_precision_loss)]
    #[must_use]
    pub fn top_mode_and_vector(
        &self,
        iterations: usize,
        perturbation: f64,
        viscous_weight: f64,
    ) -> (TopMode, Vec<[f64; 3]>) {
        let nodes = self.node_count();
        let mut v: Vec<[R; 3]> = (0..nodes)
            .map(|a| {
                let x = a as R;
                self.free_part(a, [(1.1 * x).sin(), (0.7 * x).cos(), (0.3 * x + 1.0).sin()])
            })
            .collect();
        let elastic = self.elastic();
        let base = elastic.forces(&self.displacements);
        // C v: minus the viscous forces at velocities v, in the free directions.
        let damping = |v: &[[R; 3]]| -> Vec<[R; 3]> {
            let mut element_viscous = vec![[0.0; 12]; self.elements.len()];
            elastic.viscous_forces(&self.displacements, v, &mut element_viscous);
            let mut viscous = vec![[0.0; 3]; nodes];
            elastic.gather_forces(&element_viscous, &mut viscous);
            (0..nodes)
                .map(|a| self.free_part(a, shared::vec3_scale(viscous[a], -1.0)))
                .collect()
        };
        let weight = if self.viscous {
            narrow(viscous_weight)
        } else {
            0.0
        };
        let quotient = |v: &[[R; 3]], w: &[[R; 3]]| -> f64 {
            let (numerator, denominator) = (0..nodes).fold((0.0, 0.0), |(n, d), a| {
                (
                    n + widen(shared::vec3_dot(v[a], w[a])),
                    d + widen(self.masses[a] * shared::vec3_dot(v[a], v[a])),
                )
            });
            numerator / denominator
        };
        // The vector the latest quotient was taken of, and that quotient.
        let (mut measured, mut stiffness_quotient) = (v.clone(), 0.0);
        for _ in 0..iterations {
            let largest = v
                .iter()
                .flat_map(|x| x.iter())
                .fold(0.0, |m: R, &c| m.max(c.abs()));
            if largest == 0.0 {
                break;
            }
            let scale = narrow(perturbation) / largest;
            let shifted: Vec<[R; 3]> = self
                .displacements
                .iter()
                .zip(&v)
                .map(|(&u, &x)| shared::vec3_add(u, shared::vec3_scale(x, scale)))
                .collect();
            let shifted_forces = elastic.forces(&shifted);
            // K v = −(f(u + s v) − f(u)) / s, restricted to the free directions.
            let stiffness: Vec<[R; 3]> = (0..nodes)
                .map(|a| {
                    let difference = shared::vec3_sub(shifted_forces[a], base[a]);
                    self.free_part(a, shared::vec3_scale(difference, -1.0 / scale))
                })
                .collect();
            stiffness_quotient = quotient(&v, &stiffness);
            let operator: Vec<[R; 3]> = if weight > 0.0 {
                damping(&v)
                    .iter()
                    .zip(&stiffness)
                    .map(|(&c, &k)| shared::vec3_add(k, shared::vec3_scale(c, weight)))
                    .collect()
            } else {
                stiffness
            };
            // The next iterate, M⁻¹ (K + βC) v, scaled back to a largest
            // component of 1: unscaled it grows by about ω² per iteration and
            // overflows.
            let next: Vec<[R; 3]> = (0..nodes)
                .map(|a| shared::vec3_scale(operator[a], self.inverse_masses[a]))
                .collect();
            let size = next
                .iter()
                .flat_map(|x| x.iter())
                .fold(0.0, |m: R, &c| m.max(c.abs()));
            if size == 0.0 {
                // The quotients were taken of `v`; it is the vector returned.
                measured = std::mem::take(&mut v);
                break;
            }
            let scaled = next.iter().map(|&x| shared::vec3_scale(x, 1.0 / size)).collect();
            measured = std::mem::replace(&mut v, scaled);
        }
        let mode = TopMode {
            omega_squared: stiffness_quotient,
            damping_quotient: if self.viscous {
                quotient(&measured, &damping(&measured))
            } else {
                0.0
            },
        };
        (mode, measured.iter().map(|&x| widen3(x)).collect())
    }
}

impl Executor for CpuExecutor {
    fn node_count(&self) -> usize {
        self.rest.len()
    }

    fn shortest_edge(&self) -> f64 {
        self.shortest_edge
    }

    fn epsilon(&self) -> f64 {
        widen(R::EPSILON)
    }

    fn set_state(
        &mut self,
        time: f64,
        displacements: &[[f64; 3]],
        velocities: &[[f64; 3]],
        anchors: Option<&[[f64; 3]]>,
    ) {
        let nodes = self.node_count();
        assert_eq!(displacements.len(), nodes, "one displacement per node");
        assert_eq!(velocities.len(), nodes, "one velocity per node");
        self.displacements = (0..nodes)
            .map(|a| self.free_part(a, narrow3(displacements[a])))
            .collect();
        self.velocities = (0..nodes)
            .map(|a| self.free_part(a, narrow3(velocities[a])))
            .collect();
        self.anchors = if let Some(given) = anchors {
            assert_eq!(given.len(), nodes, "one anchor per node");
            self.surface_nodes
                .iter()
                .map(|&a| narrow3(given[a as usize]))
                .collect()
        } else {
            let pose = self.pose_at(time);
            self.surface_nodes
                .iter()
                .map(|&a| {
                    let a = a as usize;
                    shared::pose_to_body(pose, shared::vec3_add(self.rest[a], self.displacements[a]))
                })
                .collect()
        };
    }

    fn set_poses(
        &mut self,
        start: f64,
        interval: f64,
        poses: &[crate::f64::Pose],
    ) -> Result<(), ObstacleError> {
        check_poses(start, interval, poses)?;
        self.pose_start = narrow(start);
        self.pose_interval = narrow(interval);
        self.poses = poses.iter().map(|&p| narrow_pose(p)).collect();
        self.track = Track::of(start, interval, poses);
        Ok(())
    }

    fn element_dilations(&mut self) {
        let mut dilations = std::mem::take(&mut self.dilations);
        self.elastic().dilations(&self.displacements, &mut dilations);
        let inverted = dilations.iter().filter(|&&d| shared::is_inverted(d)).count();
        self.inverted_element_steps += inverted as u64;
        self.dilations = dilations;
    }

    fn gather_volume_changes(&mut self) {
        let mut volume_changes = std::mem::take(&mut self.volume_changes);
        self.elastic().volume_changes(&self.dilations, &mut volume_changes);
        self.volume_changes = volume_changes;
    }

    fn nodal_pressures(&mut self) {
        let mut pressures = std::mem::take(&mut self.pressures);
        self.elastic().pressures(&self.volume_changes, &mut pressures);
        self.pressures = pressures;
    }

    fn element_forces(&mut self) {
        let mut element_forces = std::mem::take(&mut self.element_forces);
        self.elastic().element_forces(
            &self.displacements,
            &self.dilations,
            &self.pressures,
            &mut element_forces,
        );
        self.element_forces = element_forces;
        if self.viscous {
            let mut viscous = std::mem::take(&mut self.element_viscous_forces);
            self.elastic()
                .viscous_forces(&self.displacements, &self.velocities, &mut viscous);
            self.element_viscous_forces = viscous;
        }
    }

    fn gather_forces(&mut self) {
        let mut forces = std::mem::take(&mut self.elastic_forces);
        self.elastic().gather_forces(&self.element_forces, &mut forces);
        self.elastic_forces = forces;
        if self.viscous {
            let mut viscous = std::mem::take(&mut self.viscous_forces);
            self.elastic()
                .gather_forces(&self.element_viscous_forces, &mut viscous);
            self.viscous_forces = viscous;
        }
    }

    fn contact(&mut self, time: f64, dt: f64, damping: f64) {
        let pose = self.pose_at(time);
        let (step, alpha) = (narrow(dt), narrow(damping));
        let body = |a: usize| {
            shared::pose_to_body(pose, shared::vec3_add(self.rest[a], self.displacements[a]))
        };
        let mut samples = std::mem::take(&mut self.samples);
        fill(&mut samples, |i| self.sample(body(self.surface_nodes[i] as usize)));
        let mut stiffnesses = std::mem::take(&mut self.stiffnesses);
        let mut contacts = std::mem::take(&mut self.contacts);
        fill(&mut stiffnesses, |i| {
            let a = self.surface_nodes[i] as usize;
            shared::kinematic_stiffness(self.masses[a], self.inverse_masses[a], alpha, step)
        });
        let next = self.pose_at(time + dt);
        let mut predictions = std::mem::take(&mut self.predictions);
        fill(&mut predictions, |i| {
            let a = self.surface_nodes[i] as usize;
            // Where the node lands without contact, its constraints applied.
            let velocity = self.free_part(
                a,
                shared::advance_velocity(
                    self.velocities[a],
                    self.internal_force(a),
                    self.inverse_masses[a],
                    alpha,
                    step,
                ),
            );
            let displacement = shared::advance_displacement(self.displacements[a], velocity, step);
            let point = shared::vec3_add(self.rest[a], self.free_part(a, displacement));
            // How deep the predicted position is, and which grid said so.
            let (sample, fine) = self.sample_and_grid(shared::pose_to_body(next, point));
            Prediction {
                point,
                depth: sample.distance,
                fine,
            }
        });
        fill(&mut contacts, |i| {
            let a = self.surface_nodes[i] as usize;
            // The normal where the node is now, carried into the step's end
            // frame.
            let normal = shared::pose_unrotate(next, shared::pose_rotate(pose, samples[i].normal));
            shared::kinematic_contact(
                next,
                predictions[i].point,
                shared::SdfSample {
                    distance: predictions[i].depth,
                    normal,
                },
                self.anchors[i],
                stiffnesses[i],
                self.friction,
                self.constraints[a],
            )
        });
        // The step's resultant and its moment about the obstacle's body origin, where each node is at the start of
        // the step, and the work the obstacle's move over the step does against them.
        let (from, to) = (self.track.at(time), self.track.at(time + dt));
        let origin = [from.tx, from.ty, from.tz];
        let (mut resultant, mut moment) = ([0.0; 3], [0.0; 3]);
        for (i, contact) in contacts.iter().enumerate() {
            let a = self.surface_nodes[i] as usize;
            let force = widen3(contact.force);
            let at = crate::f64::vec3_add(widen3(self.rest[a]), widen3(self.displacements[a]));
            let arm = crate::f64::vec3_sub(at, origin);
            resultant = crate::f64::vec3_add(resultant, force);
            moment = crate::f64::vec3_add(moment, crate::f64::vec3_cross(arm, force));
        }
        let (moved, turn) = rigid_motion(from, to);
        self.obstacle_work +=
            crate::f64::vec3_dot(resultant, moved) + crate::f64::vec3_dot(moment, turn);
        self.since_read.moment = crate::f64::vec3_add(self.since_read.moment, moment);
        for (i, contact) in contacts.iter().enumerate() {
            self.anchors[i] = contact.anchor;
            let depth = (-samples[i].distance).max(0.0);
            self.max_penetrations[i] = self.max_penetrations[i].max(depth);
            // A node the law cannot move, held whole, takes no correction, so its depth is not read.
            if stiffnesses[i] > 0.0 {
                let predicted = (-predictions[i].depth).max(0.0);
                self.max_predictions[i] = self.max_predictions[i].max(predicted);
            }
            if contact.normal_force > 0.0 && !predictions[i].fine {
                self.coarse_corrections += 1;
            }
            for d in 0..3 {
                self.since_read.resultant[d] += widen(contact.force[d]);
            }
            self.since_read.normal += widen(contact.normal_force);
        }
        self.since_read.steps += 1;
        self.contacts = contacts;
        self.stiffnesses = stiffnesses;
        self.samples = samples;
        self.predictions = predictions;
    }

    fn integrate(&mut self, dt: f64, damping: f64) {
        let (step, alpha) = (narrow(dt), narrow(damping));
        self.previous_velocities.copy_from_slice(&self.velocities);
        let mut velocities = std::mem::take(&mut self.velocities);
        update(&mut velocities, |a, v| {
            let force = shared::vec3_add(self.internal_force(a), self.contact_force(a));
            shared::advance_velocity(v, force, self.inverse_masses[a], alpha, step)
        });
        let mut displacements = std::mem::take(&mut self.displacements);
        update(&mut displacements, |a, u| {
            shared::advance_displacement(u, velocities[a], step)
        });
        self.velocities = velocities;
        self.displacements = displacements;
    }

    fn boundary_conditions(&mut self, dt: f64, damping: f64) {
        let (step, alpha) = (narrow(dt), narrow(damping));
        let mut velocities = std::mem::take(&mut self.velocities);
        update(&mut velocities, |a, v| self.free_part(a, v));
        let mut displacements = std::mem::take(&mut self.displacements);
        update(&mut displacements, |a, u| self.free_part(a, u));
        let mut work = std::mem::take(&mut self.contact_work);
        update(&mut work, |a, w| {
            w + widen(shared::step_work(
                self.contact_force(a),
                self.previous_velocities[a],
                velocities[a],
                step,
            ))
        });
        let mut losses = std::mem::take(&mut self.damping_losses);
        update(&mut losses, |a, loss| {
            let viscous = shared::step_work(
                self.viscous_forces[a],
                self.previous_velocities[a],
                velocities[a],
                step,
            );
            loss + widen(shared::damping_loss(
                self.masses[a],
                alpha,
                self.previous_velocities[a],
                velocities[a],
                step,
            )) - widen(viscous)
        });
        self.velocities = velocities;
        self.displacements = displacements;
        self.contact_work = work;
        self.damping_losses = losses;
    }

    fn accumulate(&mut self) {
        for (sum, u) in self.displacement_sums.iter_mut().zip(&self.displacements) {
            for d in 0..3 {
                sum[d] += widen(u[d]);
            }
        }
        for (i, contact) in self.contacts.iter().enumerate() {
            self.normal_force_sums[i] += widen(contact.normal_force);
            for d in 0..3 {
                self.friction_sums[i][d] += widen(contact.friction[d]);
            }
        }
        self.accumulated_steps += 1;
    }

    fn clear_accumulators(&mut self) {
        self.displacement_sums.fill([0.0; 3]);
        self.normal_force_sums.fill(0.0);
        self.friction_sums.fill([0.0; 3]);
        self.accumulated_steps = 0;
    }

    // Step counts stay far below 2^52, where u64 → f64 starts to round.
    #[allow(clippy::cast_precision_loss)]
    fn monitors(&mut self) -> Monitors {
        let kinetic = |a: usize| widen(shared::kinetic_energy(self.masses[a], self.velocities[a]));
        let steps = self.since_read.steps;
        let mean = if steps == 0 { 0.0 } else { 1.0 / steps as f64 };
        let monitors = Monitors {
            kinetic_energy: (0..self.node_count()).map(kinetic).sum(),
            internal_energy: self.elastic().energy(&self.displacements),
            contact_kinetic_energy: self
                .surface_nodes
                .iter()
                .zip(&self.contacts)
                .filter(|(_, contact)| contact.normal_force > 0.0)
                .map(|(&a, _)| kinetic(a as usize))
                .sum(),
            contact_force: self.since_read.resultant.map(|f| f * mean),
            contact_moment: self.since_read.moment.map(|m| m * mean),
            normal_force: self.since_read.normal * mean,
            steps,
            inverted_element_steps: self.inverted_element_steps,
            contact_work: self.contact_work.iter().sum(),
            obstacle_work: self.obstacle_work,
            damping_loss: self.damping_losses.iter().sum(),
            max_penetration: self
                .max_penetrations
                .iter()
                .fold(0.0, |m: f64, &p| m.max(widen(p))),
            deepest_prediction: self
                .max_predictions
                .iter()
                .fold(0.0, |m: f64, &p| m.max(widen(p))),
            coarse_corrections: self.coarse_corrections,
        };
        self.since_read = SinceRead::default();
        monitors
    }

    fn snapshot(&mut self) -> Snapshot {
        let nodes = self.node_count();
        let mut anchors = vec![[0.0; 3]; nodes];
        let mut normal_force_sums = vec![0.0; nodes];
        let mut friction_sums = vec![[0.0; 3]; nodes];
        for (i, &a) in self.surface_nodes.iter().enumerate() {
            anchors[a as usize] = widen3(self.anchors[i]);
            normal_force_sums[a as usize] = self.normal_force_sums[i];
            friction_sums[a as usize] = self.friction_sums[i];
        }
        Snapshot {
            displacements: self.displacements.iter().map(|&u| widen3(u)).collect(),
            velocities: self.velocities.iter().map(|&v| widen3(v)).collect(),
            anchors,
            displacement_sums: self.displacement_sums.clone(),
            normal_force_sums,
            friction_sums,
            accumulated_steps: self.accumulated_steps,
        }
    }

    fn phase_outputs(&mut self) -> PhaseOutputs {
        let nodes = self.node_count();
        let mut contact_forces = vec![[0.0; 3]; nodes];
        let mut normal_forces = vec![0.0; nodes];
        for (i, &a) in self.surface_nodes.iter().enumerate() {
            contact_forces[a as usize] = widen3(self.contacts[i].force);
            normal_forces[a as usize] = widen(self.contacts[i].normal_force);
        }
        PhaseOutputs {
            dilations: self.dilations.iter().map(|&d| widen(d)).collect(),
            volume_changes: self.volume_changes.iter().map(|&v| widen(v)).collect(),
            pressures: self.pressures.iter().map(|&p| widen(p)).collect(),
            element_forces: self.element_forces.iter().map(|f| f.map(widen)).collect(),
            element_viscous_forces: self
                .element_viscous_forces
                .iter()
                .map(|f| f.map(widen))
                .collect(),
            elastic_forces: self.elastic_forces.iter().map(|&f| widen3(f)).collect(),
            viscous_forces: self.viscous_forces.iter().map(|&f| widen3(f)).collect(),
            contact_forces,
            normal_forces,
        }
    }

    fn estimate_top_mode(
        &mut self,
        iterations: usize,
        perturbation: f64,
        viscous_weight: f64,
    ) -> TopMode {
        self.top_mode_and_vector(iterations, perturbation, viscous_weight)
            .0
    }
}
