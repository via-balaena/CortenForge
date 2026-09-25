// The CPU executor at one precision. `src/cpu.rs` includes this file twice,
// with `R` as `f32` and as `f64` and `shared` as the matching shared math, so
// the two precisions run the same code and K3 compares them (plan §15a).
//
// Per-element and per-node work goes through `fill` and `update`, which run
// under rayon on native and in order on wasm32. A gather sums each node's
// incidence list in its fixed order, so no two threads add into one place
// and the result does not depend on how the work is split. Reductions for
// the monitors run in node order, in f64.

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

const fn narrow_material(m: crate::f64::Material) -> shared::Material {
    shared::Material {
        mu: narrow(m.mu),
        lambda: narrow(m.lambda),
        c2: narrow(m.c2),
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
    node_lambdas: &'a [R],
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
                self.node_lambdas[a],
            )
        });
    }

    /// Phase 4 into `element_forces`.
    fn element_forces(
        &self,
        displacements: &[[R; 3]],
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
                shared::element_pressure([a, b, c, d]),
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
        self.element_forces(displacements, &pressures, &mut element_forces);
        self.gather_forces(&element_forces, &mut forces);
        forces
    }

    /// The internal energy of `displacements`: the μ terms per element and
    /// the averaged λ term per node, `Σ_e V_e Ψ_μ + Σ_a V_a λ_a/2 (ln J_a)²`.
    fn energy(&self, displacements: &[[R; 3]]) -> f64 {
        let nodes = self.node_rest_volumes.len();
        let mut dilations = vec![0.0; self.elements.len()];
        let mut volume_changes = vec![0.0; nodes];
        self.dilations(displacements, &mut dilations);
        self.volume_changes(&dilations, &mut volume_changes);
        let elements: f64 = (0..self.elements.len())
            .map(|e| {
                widen(shared::tet4_energy_mu_terms(
                    gather12(displacements, self.elements[e]),
                    self.rest_edge_inverses[e],
                    self.rest_volumes[e],
                    self.materials[e],
                ))
            })
            .sum();
        let nodes: f64 = (0..nodes)
            .map(|a| {
                widen(
                    self.node_rest_volumes[a]
                        * shared::energy_density_lambda_term(
                            shared::nodal_dilation(volume_changes[a], self.node_rest_volumes[a]),
                            self.node_lambdas[a],
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
    node_lambdas: Vec<R>,
    constraints: Vec<[[R; 3]; 2]>,
    offsets: Vec<u32>,
    entries: Vec<u32>,
    shortest_edge: f64,

    /// The nodes on the boundary surface, the only ones that can touch the
    /// obstacle; the contact arrays are indexed by position in this list.
    surface_nodes: Vec<u32>,
    /// For each node, its position in `surface_nodes`, or `u32::MAX`.
    surface_index: Vec<u32>,

    grid: shared::SdfGridLayout,
    grid_values: Vec<R>,
    pose_start: R,
    pose_interval: R,
    poses: Vec<shared::Pose>,
    friction: R,
    penalty_scale: R,

    displacements: Vec<[R; 3]>,
    velocities: Vec<[R; 3]>,
    previous_velocities: Vec<[R; 3]>,
    dilations: Vec<R>,
    volume_changes: Vec<R>,
    pressures: Vec<R>,
    element_forces: Vec<[R; 12]>,
    elastic_forces: Vec<[R; 3]>,
    contacts: Vec<shared::ContactResponse>,
    anchors: Vec<[R; 3]>,
    stiffnesses: Vec<R>,

    max_penetrations: Vec<R>,
    contact_work: Vec<f64>,
    damping_losses: Vec<f64>,
    inverted_element_steps: u64,
    displacement_sums: Vec<[f64; 3]>,
    normal_force_sums: Vec<f64>,
    friction_sums: Vec<[f64; 3]>,
    accumulated_steps: u64,
    resultant_sum: [f64; 3],
    normal_sum: f64,
    steps_since_read: u64,
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
            node_lambdas: model.node_lambdas().iter().map(|&l| narrow(l)).collect(),
            constraints: model
                .constraints()
                .iter()
                .map(|[a, b]| [narrow3(*a), narrow3(*b)])
                .collect(),
            offsets: incidence.offsets().to_vec(),
            entries: incidence.entries().to_vec(),
            shortest_edge: shortest_edge(model),
            surface_nodes,
            surface_index,
            grid: shared::SdfGridLayout {
                origin_x: narrow(grid.origin_x),
                origin_y: narrow(grid.origin_y),
                origin_z: narrow(grid.origin_z),
                cell_size: narrow(grid.cell_size),
                size_x: grid.size_x,
                size_y: grid.size_y,
                size_z: grid.size_z,
            },
            grid_values: obstacle.values.iter().map(|&v| narrow(v)).collect(),
            pose_start: narrow(obstacle.start),
            pose_interval: narrow(obstacle.interval),
            poses: obstacle.poses.iter().map(|&p| narrow_pose(p)).collect(),
            friction: narrow(obstacle.friction),
            penalty_scale: narrow(obstacle.penalty_scale),
            displacements: vec![[0.0; 3]; nodes],
            velocities: vec![[0.0; 3]; nodes],
            previous_velocities: vec![[0.0; 3]; nodes],
            dilations: vec![0.0; model.element_count()],
            volume_changes: vec![0.0; nodes],
            pressures: vec![0.0; nodes],
            element_forces: vec![[0.0; 12]; model.element_count()],
            elastic_forces: vec![[0.0; 3]; nodes],
            contacts: vec![empty_contact; surface_count],
            anchors,
            stiffnesses: vec![0.0; surface_count],
            max_penetrations: vec![0.0; surface_count],
            contact_work: vec![0.0; nodes],
            damping_losses: vec![0.0; nodes],
            inverted_element_steps: 0,
            displacement_sums: vec![[0.0; 3]; nodes],
            normal_force_sums: vec![0.0; surface_count],
            friction_sums: vec![[0.0; 3]; surface_count],
            accumulated_steps: 0,
            resultant_sum: [0.0; 3],
            normal_sum: 0.0,
            steps_since_read: 0,
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
            node_lambdas: &self.node_lambdas,
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

    /// The obstacle's distance and normal at a body-frame point: the seven
    /// probes of the shared lookup, fetched from the grid.
    fn sample(&self, point: [R; 3]) -> shared::SdfSample {
        let mut values = [0.0; 7];
        for (probe, value) in (0_u32..).zip(values.iter_mut()) {
            let coordinate = shared::sdf_probe_coordinate(point, self.grid, probe);
            let corners = shared::sdf_cell_corners(coordinate, self.grid)
                .map(|index| self.grid_values[index as usize]);
            *value = shared::sdf_trilinear(coordinate, corners);
        }
        shared::sdf_combine(values, self.grid)
    }
}

/// The shortest element edge at rest.
fn shortest_edge(model: &ExplicitModel) -> f64 {
    let p = model.rest_positions();
    model
        .elements()
        .iter()
        .flat_map(|e| {
            [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)].map(|(i, j)| {
                let (a, b) = (p[e[i] as usize], p[e[j] as usize]);
                ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
            })
        })
        .fold(f64::INFINITY, f64::min)
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

    fn penalty_scale(&self) -> f64 {
        widen(self.penalty_scale)
    }

    fn friction(&self) -> f64 {
        widen(self.friction)
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
        self.elastic()
            .element_forces(&self.displacements, &self.pressures, &mut element_forces);
        self.element_forces = element_forces;
    }

    fn gather_forces(&mut self) {
        let mut forces = std::mem::take(&mut self.elastic_forces);
        self.elastic().gather_forces(&self.element_forces, &mut forces);
        self.elastic_forces = forces;
    }

    fn contact(&mut self, time: f64, dt: f64) {
        let pose = self.pose_at(time);
        let step = narrow(dt);
        let mut stiffnesses = std::mem::take(&mut self.stiffnesses);
        fill(&mut stiffnesses, |i| {
            let a = self.surface_nodes[i] as usize;
            shared::penalty_stiffness(self.masses[a], step, self.penalty_scale)
        });
        let mut contacts = std::mem::take(&mut self.contacts);
        fill(&mut contacts, |i| {
            let a = self.surface_nodes[i] as usize;
            let world = shared::vec3_add(self.rest[a], self.displacements[a]);
            let body = shared::pose_to_body(pose, world);
            shared::obstacle_contact(
                pose,
                body,
                self.sample(body),
                self.anchors[i],
                stiffnesses[i],
                self.friction,
            )
        });
        for (i, contact) in contacts.iter().enumerate() {
            self.anchors[i] = contact.anchor;
            if stiffnesses[i] > 0.0 {
                let depth = contact.normal_force / stiffnesses[i];
                self.max_penetrations[i] = self.max_penetrations[i].max(depth);
            }
            for d in 0..3 {
                self.resultant_sum[d] += widen(contact.force[d]);
            }
            self.normal_sum += widen(contact.normal_force);
        }
        self.steps_since_read += 1;
        self.contacts = contacts;
        self.stiffnesses = stiffnesses;
    }

    fn integrate(&mut self, dt: f64, damping: f64) {
        let (step, alpha) = (narrow(dt), narrow(damping));
        self.previous_velocities.copy_from_slice(&self.velocities);
        let mut velocities = std::mem::take(&mut self.velocities);
        update(&mut velocities, |a, v| {
            let force = shared::vec3_add(self.elastic_forces[a], self.contact_force(a));
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
            loss + widen(shared::damping_loss(
                self.masses[a],
                alpha,
                self.previous_velocities[a],
                velocities[a],
                step,
            ))
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
        let steps = self.steps_since_read;
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
            contact_force: self.resultant_sum.map(|f| f * mean),
            normal_force: self.normal_sum * mean,
            steps,
            inverted_element_steps: self.inverted_element_steps,
            contact_work: self.contact_work.iter().sum(),
            damping_loss: self.damping_losses.iter().sum(),
            max_penetration: self
                .max_penetrations
                .iter()
                .fold(0.0, |m: f64, &p| m.max(widen(p))),
        };
        self.resultant_sum = [0.0; 3];
        self.normal_sum = 0.0;
        self.steps_since_read = 0;
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
            elastic_forces: self.elastic_forces.iter().map(|&f| widen3(f)).collect(),
            contact_forces,
            normal_forces,
        }
    }

    // Node indices feed a deterministic starting vector; precision loss in
    // `usize → R` only changes which starting vector it is.
    #[allow(clippy::cast_precision_loss)]
    fn elastic_rayleigh_quotient(&mut self, iterations: usize, perturbation: f64) -> f64 {
        let nodes = self.node_count();
        let mut v: Vec<[R; 3]> = (0..nodes)
            .map(|a| {
                let x = a as R;
                self.free_part(a, [(1.1 * x).sin(), (0.7 * x).cos(), (0.3 * x + 1.0).sin()])
            })
            .collect();
        let elastic = self.elastic();
        let base = elastic.forces(&self.displacements);
        let mut quotient = 0.0;
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
            let (numerator, denominator) = (0..nodes).fold((0.0, 0.0), |(n, d), a| {
                (
                    n + widen(shared::vec3_dot(v[a], stiffness[a])),
                    d + widen(self.masses[a] * shared::vec3_dot(v[a], v[a])),
                )
            });
            quotient = numerator / denominator;
            // The next iterate, M⁻¹ K v, scaled back to a largest component of 1:
            // unscaled it grows by about ω² per iteration and overflows.
            let next: Vec<[R; 3]> = (0..nodes)
                .map(|a| shared::vec3_scale(stiffness[a], self.inverse_masses[a]))
                .collect();
            let size = next
                .iter()
                .flat_map(|x| x.iter())
                .fold(0.0, |m: R, &c| m.max(c.abs()));
            if size == 0.0 {
                break;
            }
            v = next.iter().map(|&x| shared::vec3_scale(x, 1.0 / size)).collect();
        }
        quotient
    }
}
