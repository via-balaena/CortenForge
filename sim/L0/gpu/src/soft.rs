//! The explicit soft-body solver's executor on the GPU (recon §15g step 4,
//! §17b): `sim-soft-explicit`'s [`Executor`] over its model and obstacle, at
//! f32, with the shared math's WGSL for the physics.
//!
//! It runs the CPU executor's phases, one entry point each, over the same
//! arrays. What is particular to the GPU:
//!
//! - **One pass a step.** Each phase appends its dispatches to a pending
//!   list, held in the loop's order. The list is recorded as one compute pass,
//!   in one [`Recorder`] step, when a phase does not follow the last one
//!   pending, when the step's `dt` or damping changes, and before any read,
//!   write or [`Executor::clear_accumulators`]. In the stepping loop that is
//!   once a step: a pass costs 12–22 µs on Metal beyond its dispatches.
//! - **The pose, on the host.** `contact` interpolates the obstacle's pose at
//!   the step's start and end with the shared f32 math, as the CPU executor
//!   does, and the step carries both. So [`Executor::set_poses`] writes
//!   nothing to the device.
//! - **Sums over nodes.** Metal compiles with fast math, which drops a
//!   compensated sum's error term, so the device keeps none. Each sum is a
//!   fixed f32 tree, per step into a row of the step log or per workgroup at a
//!   read, and the host adds the rows and partials at f64 (`soft/log.rs`). Maxima
//!   are `max` trees, and counts u32 atomics into a 64-bit pair.
//! - **Reads.** Rows, window sums and partials come back only at the trait's
//!   reads, each one [`Recorder::read_many`]. The internal energy read and the
//!   step estimate run the phases on scratch arrays, so they change neither
//!   the step's outputs nor its counts.

mod kernels;
mod log;
#[cfg(test)]
mod tests;

use bytemuck::{Pod, Zeroable};
use sim_soft_explicit::ExplicitModel;
use sim_soft_explicit::executor::{
    Executor, Monitors, Obstacle, ObstacleError, PhaseOutputs, Snapshot, TopMode, check_obstacle,
    check_poses, rigid_motion,
};
use sim_soft_explicit::f32 as shared;
use wgpu::util::DeviceExt;

use crate::context::GpuContext;
use crate::submit::{Recorder, Recording, values};
use kernels::binding::{
    ANCHORS, BASE_FORCES, CONTACTS, COUNTERS, DAMPINGS, DILATIONS, DISPLACEMENTS, ELASTIC_FORCES,
    ELEMENT_FORCES, ELEMENTS, ENTRIES, FINE_MAP, FINE_VALUES, GRID_VALUES, LOG_ROWS, MAGNITUDES,
    MAXIMA, MEASURED, NEXT_VECTOR, NODE_FORCES, NODES, OFFSETS, PARTIALS, PRESSURES,
    PREVIOUS_VELOCITIES, QUOTIENTS, REDUCTION, SCALARS, SHIFTED, SHIFTED_FORCES, START_VECTOR,
    SURFACE_NODES, TERMS, VECTOR, VELOCITIES, VISCOUS_AT, VISCOUS_FORCES, VOLUME_CHANGES, WINDOW,
};
use kernels::{Dispatch, Kernel, Kernels, Shared};
use log::{BOUNDARY_ROW, CONTACT_ROW, Motion, Totals};

/// Items a reduction's partial covers (`soft.wgsl`'s `TREE`).
const TREE: u32 = 256;

/// Steps a submit's ring holds: a step is one pass, so a submit at
/// [`crate::submit::STEP_PASS_CAP`] passes is this many steps.
const RING_SLOTS: u32 = crate::submit::STEP_PASS_CAP;

/// Rows the step log starts with; it doubles when full.
const LOG_ROWS_AT_START: u32 = 64;

/// A node's constants (`soft.wgsl`'s `Node`).
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Node {
    rest: [f32; 3],
    arm: [f32; 3],
    mass: f32,
    inverse_mass: f32,
    rest_volume: f32,
    lambda: f32,
    constraints: [[f32; 3]; 2],
    surface: u32,
}

/// An element's constants (`soft.wgsl`'s `Element`).
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Element {
    nodes: [u32; 4],
    edge_inverse: [f32; 9],
    rest_volume: f32,
    stabilization: f32,
    /// μ, λ, C₂, the viscosity and the density (`Material`'s order).
    material: [f32; 5],
}

/// A grid's layout (`SdfGridLayout`).
#[repr(C)]
#[derive(Clone, Copy, Default, Pod, Zeroable)]
struct Layout {
    origin: [f32; 3],
    cell_size: f32,
    size: [u32; 3],
}

/// The model's and the obstacle's constants (`soft.wgsl`'s `Constants`),
/// each layout at a 16-byte boundary as a uniform needs.
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Constants {
    nodes: u32,
    elements: u32,
    surface: u32,
    viscous: u32,
    grid: Layout,
    _grid_end: u32,
    fine: Layout,
    _fine_end: u32,
    has_fine: u32,
    friction: f32,
    _end: [u32; 2],
}

/// What a step carries to its passes (`soft.wgsl`'s `StepValues`), each pose
/// at a 16-byte boundary as a uniform needs.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Pod, Zeroable)]
struct StepValues {
    dt: f32,
    damping: f32,
    contact_row: u32,
    boundary_row: u32,
    start: [f32; 7],
    _start_end: f32,
    end: [f32; 7],
    _end_end: f32,
    perturbation: f32,
    weight: f32,
    _end: [f32; 2],
}

const _: () = assert!(std::mem::size_of::<Node>() == 68);
const _: () = assert!(std::mem::size_of::<Element>() == 80);
const _: () = assert!(std::mem::size_of::<Constants>() == 96);
const _: () = assert!(std::mem::size_of::<StepValues>() == 96);

/// A reduction (`soft.wgsl`'s `Reduction`).
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Reduction {
    items: u32,
    components: u32,
    operation: u32,
    row: u32,
}

/// A reduction's operations and rows (`soft.wgsl`'s constants).
const SUM: u32 = 0;
const MAX: u32 = 1;
const ON_CONTACT_ROW: u32 = u32::MAX;
const ON_BOUNDARY_ROW: u32 = u32::MAX - 1;

/// The power iteration's scalars (`soft.wgsl`'s constants).
const LARGEST: u32 = 1;
const SIZE: u32 = 2;
const TAKEN: usize = 4;
const SCALARS_LEN: usize = 5;

/// The phases of a step, in the loop's order, and the window sums after them.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Phase {
    Dilations,
    VolumeChanges,
    Pressures,
    ElementForces,
    GatherForces,
    Contact,
    Integrate,
    BoundaryConditions,
    Accumulate,
}

/// The phases appended since the last recording, and what they carry.
#[derive(Default)]
struct Pending {
    phases: Vec<Phase>,
    values: StepValues,
    /// The step's `dt` and damping, once a phase that takes them is appended.
    timing: Option<[f32; 2]>,
}

impl Pending {
    /// Whether `phase`, taking `timing`, needs what is pending recorded first:
    /// it does not follow the last phase, or the timing changed.
    fn must_record_before(&self, phase: Phase, timing: Option<[f32; 2]>) -> bool {
        let changed = match (self.timing, timing) {
            (Some(held), Some(given)) => held.map(f32::to_bits) != given.map(f32::to_bits),
            _ => false,
        };
        self.phases.last().is_some_and(|&last| phase <= last) || changed
    }
}

/// Why a [`GpuExecutor`] could not be built.
#[derive(Debug)]
pub enum SoftError {
    /// The obstacle was rejected.
    Obstacle(ObstacleError),
    /// An array is larger than the device binds.
    TooLarge {
        /// Which array.
        what: &'static str,
        /// Its size.
        bytes: u64,
        /// The largest the device binds.
        limit: u64,
    },
}

impl std::fmt::Display for SoftError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Obstacle(err) => write!(f, "{err}"),
            Self::TooLarge { what, bytes, limit } => write!(
                f,
                "the {what} take {bytes} bytes; the device binds at most {limit}"
            ),
        }
    }
}

impl std::error::Error for SoftError {}

impl From<ObstacleError> for SoftError {
    fn from(err: ObstacleError) -> Self {
        Self::Obstacle(err)
    }
}

/// A pose track at f64, as the CPU executor keeps it: the obstacle's motion
/// over each step, for its work, is read from it.
#[derive(Clone, Debug)]
struct Track {
    start: f64,
    interval: f64,
    poses: Vec<sim_soft_explicit::f64::Pose>,
}

impl Track {
    // A pose track is a few thousand samples.
    #[allow(clippy::cast_possible_truncation)]
    fn at(&self, time: f64) -> sim_soft_explicit::f64::Pose {
        let span = sim_soft_explicit::f64::pose_sample_span(
            time,
            self.start,
            self.interval,
            self.poses.len() as u32,
        );
        sim_soft_explicit::f64::pose_interpolate(
            self.poses[span.lower as usize],
            self.poses[span.upper as usize],
            span.fraction,
        )
    }
}

/// The pose track at f32, which the contact phase reads, and at f64, which
/// the obstacle's work is read from.
struct Tracks {
    start: f32,
    interval: f32,
    poses: Vec<shared::Pose>,
    exact: Track,
}

impl Tracks {
    // f64 → f32 rounds, as the CPU executor at f32 narrows its boundary values.
    #[allow(clippy::cast_possible_truncation)]
    fn of(start: f64, interval: f64, poses: &[sim_soft_explicit::f64::Pose]) -> Self {
        Self {
            start: start as f32,
            interval: interval as f32,
            poses: poses.iter().map(|&p| narrow_pose(p)).collect(),
            exact: Track {
                start,
                interval,
                poses: poses.to_vec(),
            },
        }
    }

    /// The pose at `time`, as the CPU executor at f32 interpolates it.
    // f64 → f32 rounds, as the CPU executor at f32 narrows its boundary values.
    #[allow(clippy::cast_possible_truncation)]
    fn at(&self, time: f64) -> shared::Pose {
        let span = shared::pose_sample_span(
            time as f32,
            self.start,
            self.interval,
            self.poses.len() as u32,
        );
        shared::pose_interpolate(
            self.poses[span.lower as usize],
            self.poses[span.upper as usize],
            span.fraction,
        )
    }
}

// f64 → f32 rounds, as the CPU executor at f32 narrows its boundary values.
#[allow(clippy::cast_possible_truncation)]
const fn narrow_pose(p: sim_soft_explicit::f64::Pose) -> shared::Pose {
    shared::Pose {
        qw: p.qw as f32,
        qx: p.qx as f32,
        qy: p.qy as f32,
        qz: p.qz as f32,
        tx: p.tx as f32,
        ty: p.ty as f32,
        tz: p.tz as f32,
    }
}

const fn pose_values(p: shared::Pose) -> [f32; 7] {
    [p.qw, p.qx, p.qy, p.qz, p.tx, p.ty, p.tz]
}

// f64 → f32 rounds, as the CPU executor at f32 narrows its boundary values.
#[allow(clippy::cast_possible_truncation)]
const fn narrow3(v: [f64; 3]) -> [f32; 3] {
    [v[0] as f32, v[1] as f32, v[2] as f32]
}

fn widen3(v: [f32; 3]) -> [f64; 3] {
    v.map(f64::from)
}

/// The window sums, at f64 on the host; each `monitors` or `snapshot` adds
/// the device's f32 sums since the last.
struct Window {
    displacement_sums: Vec<[f64; 3]>,
    normal_force_sums: Vec<f64>,
    friction_sums: Vec<[f64; 3]>,
    steps: u64,
    /// Whether steps were added on the device since its sums were read.
    unread: bool,
}

/// The step log's rows on the device: a buffer, how many it holds, and how
/// many are written since the last `monitors` or `snapshot`.
struct Rows {
    buffer: wgpu::Buffer,
    capacity: u32,
    written: u32,
}

/// The node constants the host needs: to project a state, and to anchor a
/// surface node where it sits.
struct Host {
    rest: Vec<[f32; 3]>,
    inverse_masses: Vec<f32>,
    constraints: Vec<[[f32; 3]; 2]>,
    surface_nodes: Vec<u32>,
    /// The rest positions' centroid: the device takes each step's moment
    /// about it.
    centroid: [f64; 3],
}

impl Host {
    /// `v` without a node's held and constrained directions.
    fn free_part(&self, node: usize, v: [f32; 3]) -> [f32; 3] {
        if self.inverse_masses[node] == 0.0 {
            [0.0; 3]
        } else {
            let [first, second] = self.constraints[node];
            shared::constrain(v, first, second)
        }
    }
}

/// The buffers read back or written after construction.
struct Buffers {
    constants: wgpu::Buffer,
    displacements: wgpu::Buffer,
    velocities: wgpu::Buffer,
    anchors: wgpu::Buffer,
    dilations: wgpu::Buffer,
    volume_changes: wgpu::Buffer,
    pressures: wgpu::Buffer,
    element_forces: wgpu::Buffer,
    element_viscous_forces: wgpu::Buffer,
    elastic_forces: wgpu::Buffer,
    viscous_forces: wgpu::Buffer,
    contacts: wgpu::Buffer,
    counters: wgpu::Buffer,
    window: wgpu::Buffer,
    element_energy_partials: wgpu::Buffer,
    node_energy_partials: wgpu::Buffer,
    maxima_partials: wgpu::Buffer,
    scalars: wgpu::Buffer,
    quotient_partials: wgpu::Buffer,
    damping_partials: wgpu::Buffer,
    /// What a log row's reduction binds, to bind it again when the log grows.
    contact_partials: wgpu::Buffer,
    contact_reduction: wgpu::Buffer,
    boundary_partials: wgpu::Buffer,
    boundary_reduction: wgpu::Buffer,
}

/// The dispatches each piece of work records, in order.
struct Programs {
    /// Each phase's, by [`Phase`]; a contact and a boundary phase end on their
    /// log row's reduction.
    phases: [Vec<Dispatch>; 9],
    /// The monitors' read: the internal energy's phases on scratch arrays, the
    /// energies and the maxima's partials.
    read: Vec<Dispatch>,
    /// The estimate's start, and the base forces on scratch arrays.
    estimate_start: Vec<Dispatch>,
    /// An iteration up to its stiffness quotient's partials.
    iteration_head: Vec<Dispatch>,
    /// The viscous forces at velocities `v`, for a viscous weight.
    viscous_at_vector: Vec<Dispatch>,
    /// An iteration from the next iterate to its scaling.
    iteration_tail: Vec<Dispatch>,
    /// The damping quotient's terms and partials, for a viscous material.
    estimate_damping: Vec<Dispatch>,
}

/// The explicit solver's state on the GPU, at f32.
///
/// Built from a lowered [`ExplicitModel`] and an [`Obstacle`]; stepped by
/// [`sim_soft_explicit::stepping::Stepper`] through the [`Executor`] phases.
pub struct GpuExecutor {
    device: wgpu::Device,
    recorder: Recorder<StepValues>,
    kernels: Kernels,
    buffers: Buffers,
    programs: Programs,
    pending: Pending,
    contact_rows: Rows,
    boundary_rows: Rows,
    /// The obstacle's motion over the step of each contact row not yet read.
    motions: Vec<Motion>,
    totals: Totals,
    window: Window,
    host: Host,
    tracks: Tracks,
    node_count: u32,
    element_count: u32,
    surface_count: u32,
    shortest_edge: f64,
    viscous: bool,
    /// Record each phase as its own pass (the one-pass gate's comparison).
    #[cfg(test)]
    pass_per_phase: bool,
}

/// Workgroups of partials for `items`.
const fn blocks(items: u32) -> u32 {
    items.div_ceil(TREE)
}

/// Bytes of `count` values of `T`, as a buffer size.
const fn bytes<T>(count: u32) -> u64 {
    count as u64 * std::mem::size_of::<T>() as u64
}

/// Makes the executor's buffers, each checked against what the device binds.
struct Maker<'a> {
    device: &'a wgpu::Device,
    limit: u64,
}

impl Maker<'_> {
    const USAGE: wgpu::BufferUsages = wgpu::BufferUsages::STORAGE
        .union(wgpu::BufferUsages::COPY_SRC)
        .union(wgpu::BufferUsages::COPY_DST);

    const fn check(&self, what: &'static str, bytes: u64) -> Result<(), SoftError> {
        if bytes > self.limit {
            return Err(SoftError::TooLarge {
                what,
                bytes,
                limit: self.limit,
            });
        }
        Ok(())
    }

    /// A buffer holding `data`; at least 16 bytes, so an empty array binds.
    fn filled<T: Pod>(&self, what: &'static str, data: &[T]) -> Result<wgpu::Buffer, SoftError> {
        let mut contents = bytemuck::cast_slice::<T, u8>(data).to_vec();
        self.check(what, contents.len() as u64)?;
        contents.resize(contents.len().max(16), 0);
        Ok(self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(what),
                contents: &contents,
                usage: Self::USAGE,
            }))
    }

    /// A buffer of `bytes` zeros; at least 16.
    fn zeroed(&self, what: &'static str, bytes: u64) -> Result<wgpu::Buffer, SoftError> {
        self.check(what, bytes)?;
        Ok(self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(what),
            size: bytes.max(16).next_multiple_of(4),
            usage: Self::USAGE,
            mapped_at_creation: false,
        }))
    }

    /// A reduction's uniform.
    fn reduction(&self, what: &'static str, reduction: Reduction) -> wgpu::Buffer {
        self.device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some(what),
                contents: bytemuck::bytes_of(&reduction),
                usage: wgpu::BufferUsages::UNIFORM,
            })
    }
}

impl GpuExecutor {
    /// An executor for `model` against `obstacle` on `ctx`'s device, at rest.
    ///
    /// # Errors
    /// [`SoftError::Obstacle`] naming the first problem with `obstacle`, or
    /// [`SoftError::TooLarge`] for an array larger than the device binds.
    // One constructor lays out every buffer and dispatch; split, each piece
    // would take most of the others as arguments.
    #[allow(clippy::too_many_lines, clippy::cast_possible_truncation)]
    pub fn new(
        ctx: &GpuContext,
        model: &ExplicitModel,
        obstacle: &Obstacle,
    ) -> Result<Self, SoftError> {
        check_obstacle(obstacle)?;
        let device = &ctx.device;
        let limits = device.limits();
        let make = Maker {
            device,
            limit: u64::from(limits.max_storage_buffer_binding_size).min(limits.max_buffer_size),
        };
        // A model indexes its nodes by u32, and its element incidence entries,
        // four an element, fit a u32 (`ExplicitModel::new`).
        let (nodes, elements) = (model.node_count() as u32, model.element_count() as u32);

        // The host's copies, narrowed as the CPU executor at f32 narrows them.
        let surface = model.surface_incidence();
        let surface_nodes: Vec<u32> = (0..nodes)
            .filter(|&a| !surface.of(a as usize).is_empty())
            .collect();
        let surface_count = surface_nodes.len() as u32;
        let mut surface_index = vec![u32::MAX; nodes as usize];
        for (i, &a) in (0_u32..).zip(&surface_nodes) {
            surface_index[a as usize] = i;
        }
        let rest64 = model.rest_positions();
        let centroid = rest64
            .iter()
            .fold([0.0; 3], |sum, &p| sim_soft_explicit::f64::vec3_add(sum, p))
            .map(|sum| sum / f64::from(nodes));
        let host = Host {
            rest: rest64.iter().map(|&p| narrow3(p)).collect(),
            inverse_masses: model
                .node_masses()
                .iter()
                .zip(model.held())
                .map(|(&m, &held)| shared::inverse_mass(m as f32, held))
                .collect(),
            constraints: model
                .constraints()
                .iter()
                .map(|[a, b]| [narrow3(*a), narrow3(*b)])
                .collect(),
            surface_nodes,
            centroid,
        };
        let node_table: Vec<Node> = (0..nodes as usize)
            .map(|a| Node {
                rest: host.rest[a],
                arm: narrow3(sim_soft_explicit::f64::vec3_sub(rest64[a], centroid)),
                mass: model.node_masses()[a] as f32,
                inverse_mass: host.inverse_masses[a],
                rest_volume: model.node_rest_volumes()[a] as f32,
                lambda: (model.node_lambdas()[a] - model.node_stabilizations()[a]) as f32,
                constraints: host.constraints[a],
                surface: surface_index[a],
            })
            .collect();
        let element_table: Vec<Element> = (0..elements as usize)
            .map(|e| {
                let m = model.materials()[e];
                Element {
                    nodes: model.elements()[e],
                    edge_inverse: model.rest_edge_inverses()[e].map(|x| x as f32),
                    rest_volume: model.rest_volumes()[e] as f32,
                    stabilization: model.element_stabilizations()[e] as f32,
                    material: [m.mu, m.lambda, m.c2, m.viscosity, m.density].map(|x| x as f32),
                }
            })
            .collect();
        let viscous = model.materials().iter().any(|m| m.viscosity > 0.0);
        let tracks = Tracks::of(obstacle.start, obstacle.interval, &obstacle.poses);
        // Each surface node starts anchored where it sits against the track's
        // first pose, as the CPU executor starts it.
        let first = tracks.poses[0];
        let anchors: Vec<[f32; 3]> = host
            .surface_nodes
            .iter()
            .map(|&a| shared::pose_to_body(first, host.rest[a as usize]))
            .collect();
        // The start vector of every estimate, formed as the CPU executor forms
        // it: node indices feed it, and their rounding only changes which
        // vector it is.
        #[allow(clippy::cast_precision_loss)]
        let start_vector: Vec<[f32; 3]> = (0..nodes as usize)
            .map(|a| {
                let x = a as f32;
                host.free_part(a, [(1.1 * x).sin(), (0.7 * x).cos(), (0.3 * x + 1.0).sin()])
            })
            .collect();
        let layout = |grid: sim_soft_explicit::f64::SdfGridLayout| Layout {
            origin: [grid.origin_x, grid.origin_y, grid.origin_z].map(|x| x as f32),
            cell_size: grid.cell_size as f32,
            size: [grid.size_x, grid.size_y, grid.size_z],
        };
        let constants = Constants {
            nodes,
            elements,
            surface: surface_count,
            viscous: u32::from(viscous),
            grid: layout(obstacle.grid),
            _grid_end: 0,
            fine: obstacle
                .fine
                .as_ref()
                .map_or_else(Layout::default, |f| layout(f.grid)),
            _fine_end: 0,
            has_fine: u32::from(obstacle.fine.is_some()),
            friction: obstacle.friction as f32,
            _end: [0; 2],
        };
        let narrowed = |values: &[f64]| values.iter().map(|&v| v as f32).collect::<Vec<f32>>();

        // The model, the obstacle and the state.
        let constants_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("soft constants"),
            contents: bytemuck::bytes_of(&constants),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let incidence = model.element_incidence();
        let node_buffer = make.filled("node constants", &node_table)?;
        let element_buffer = make.filled("element constants", &element_table)?;
        let offsets = make.filled("incidence offsets", incidence.offsets())?;
        let entries = make.filled("incidence entries", incidence.entries())?;
        let surface_buffer = make.filled("surface nodes", &host.surface_nodes)?;
        let grid_values = make.filled("grid values", &narrowed(&obstacle.values))?;
        let (fine_map, fine_values) = match &obstacle.fine {
            Some(fine) => (
                make.filled("fine grid map", &fine.map)?,
                make.filled("fine grid values", &narrowed(&fine.values))?,
            ),
            None => (
                make.zeroed("fine grid map", 0)?,
                make.zeroed("fine grid values", 0)?,
            ),
        };
        let vectors = |what| make.zeroed(what, bytes::<[f32; 3]>(nodes));
        let displacements = vectors("displacements")?;
        let velocities = vectors("velocities")?;
        let previous_velocities = vectors("previous velocities")?;
        let anchor_buffer = make.filled("anchors", &anchors)?;
        let start_buffer = make.filled("start vector", &start_vector)?;

        // The step's outputs, and scratch arrays for a read and an estimate.
        let per_element = |what| make.zeroed(what, bytes::<f32>(elements));
        let per_node = |what| make.zeroed(what, bytes::<f32>(nodes));
        let element_vectors = |what| make.zeroed(what, bytes::<[f32; 12]>(elements));
        let dilations = per_element("dilations")?;
        let volume_changes = per_node("volume changes")?;
        let pressures = per_node("pressures")?;
        let element_forces = element_vectors("element forces")?;
        let element_viscous_forces = element_vectors("element viscous forces")?;
        let elastic_forces = vectors("elastic forces")?;
        let viscous_forces = vectors("viscous forces")?;
        let contacts = make.zeroed("contacts", bytes::<[f32; 7]>(surface_count))?;
        let maxima = make.zeroed("maxima", bytes::<[f32; 2]>(surface_count))?;
        let counters = make.zeroed("counters", bytes::<u32>(4))?;
        let scratch_counters = make.zeroed("scratch counters", bytes::<u32>(4))?;
        let window = make.zeroed("window sums", bytes::<f32>(3 * nodes + 4 * surface_count))?;
        let scratch_dilations = per_element("scratch dilations")?;
        let scratch_volume_changes = per_node("scratch volume changes")?;
        let scratch_pressures = per_node("scratch pressures")?;
        let scratch_element_forces = element_vectors("scratch element forces")?;

        // The reductions: terms per item, partials per workgroup.
        let terms =
            |what, items: u32, components: u32| make.zeroed(what, bytes::<f32>(items * components));
        let partials = |what, items: u32, components: u32| {
            make.zeroed(what, bytes::<f32>(blocks(items) * components))
        };
        let contact_terms = terms("contact terms", surface_count, 7)?;
        let contact_partials = partials("contact partials", surface_count, 7)?;
        let boundary_terms = terms("boundary terms", nodes, 2)?;
        let boundary_partials = partials("boundary partials", nodes, 2)?;
        let element_energy_terms = terms("element energies", elements, 1)?;
        let element_energy_partials = partials("element energy partials", elements, 1)?;
        let node_energy_terms = terms("node energies", nodes, 3)?;
        let node_energy_partials = partials("node energy partials", nodes, 3)?;
        let maxima_partials = partials("maxima partials", surface_count, 2)?;
        let reduction = |what, items, components, operation, row| {
            make.reduction(
                what,
                Reduction {
                    items,
                    components,
                    operation,
                    row,
                },
            )
        };
        let contact_reduction = reduction("contact row", surface_count, 7, SUM, ON_CONTACT_ROW);
        let boundary_reduction = reduction("boundary row", nodes, 2, SUM, ON_BOUNDARY_ROW);
        let element_energy_reduction = reduction("element energies", elements, 1, SUM, 0);
        let node_energy_reduction = reduction("node energies", nodes, 3, SUM, 0);
        let maxima_reduction = reduction("maxima", surface_count, 2, MAX, 0);

        // The estimate.
        let scalars = make.zeroed("estimate scalars", bytes::<f32>(SCALARS_LEN as u32))?;
        let vector = vectors("estimate vector")?;
        let measured = vectors("estimate measured")?;
        let next_vector = vectors("estimate next")?;
        let base_forces = vectors("estimate base forces")?;
        let shifted = vectors("estimate shifted")?;
        let shifted_forces = vectors("estimate shifted forces")?;
        let viscous_at = vectors("estimate viscous forces")?;
        let magnitudes = per_node("estimate magnitudes")?;
        let magnitude_partials = partials("estimate magnitude partials", nodes, 1)?;
        let quotients = terms("stiffness quotients", nodes, 2)?;
        let quotient_partials = partials("stiffness quotient partials", nodes, 2)?;
        let dampings = terms("damping quotients", nodes, 2)?;
        let damping_partials = partials("damping quotient partials", nodes, 2)?;
        let largest_reduction = reduction("estimate largest", nodes, 1, MAX, LARGEST);
        let size_reduction = reduction("estimate size", nodes, 1, MAX, SIZE);
        let quotient_reduction = reduction("stiffness quotients", nodes, 2, SUM, 0);
        let damping_reduction = reduction("damping quotients", nodes, 2, SUM, 0);

        let contact_rows = Rows {
            buffer: make.zeroed(
                "contact rows",
                bytes::<[f32; CONTACT_ROW]>(LOG_ROWS_AT_START),
            )?,
            capacity: LOG_ROWS_AT_START,
            written: 0,
        };
        let boundary_rows = Rows {
            buffer: make.zeroed(
                "boundary rows",
                bytes::<[f32; BOUNDARY_ROW]>(LOG_ROWS_AT_START),
            )?,
            capacity: LOG_ROWS_AT_START,
            written: 0,
        };

        let recorder = Recorder::<StepValues>::with_step_values(ctx, RING_SLOTS);
        let kernels = Kernels::new(ctx);
        // Cannot fail: the recorder is built with `with_step_values`.
        let Some(step_values) = recorder.step_values_binding() else {
            unreachable!("a recorder built with step values binds them");
        };
        let shared_bindings = Shared {
            constants: &constants_buffer,
            step_values,
        };
        let bind = |kernel, items, buffers: &[(u32, &wgpu::Buffer)]| {
            kernels.bind(device, &shared_bindings, kernel, items, buffers)
        };

        // Phases 1–2 over `displacements` into the given arrays, counting
        // inverted elements into `counted`.
        let volume_phases = |displacements: &wgpu::Buffer,
                             dilations: &wgpu::Buffer,
                             volume_changes: &wgpu::Buffer,
                             counted: &wgpu::Buffer| {
            [
                bind(
                    Kernel::ElementDilations,
                    elements,
                    &[
                        (ELEMENTS, &element_buffer),
                        (DISPLACEMENTS, displacements),
                        (DILATIONS, dilations),
                        (COUNTERS, counted),
                    ],
                ),
                bind(
                    Kernel::GatherVolumeChanges,
                    nodes,
                    &[
                        (ELEMENTS, &element_buffer),
                        (OFFSETS, &offsets),
                        (ENTRIES, &entries),
                        (DILATIONS, dilations),
                        (VOLUME_CHANGES, volume_changes),
                    ],
                ),
            ]
        };
        // Phases 3–5's elastic part, over phases 1–2's arrays, into `forces`.
        let force_phases = |displacements: &wgpu::Buffer,
                            dilations: &wgpu::Buffer,
                            volume_changes: &wgpu::Buffer,
                            pressures: &wgpu::Buffer,
                            element_forces: &wgpu::Buffer,
                            forces: &wgpu::Buffer| {
            [
                bind(
                    Kernel::NodalPressures,
                    nodes,
                    &[
                        (NODES, &node_buffer),
                        (VOLUME_CHANGES, volume_changes),
                        (PRESSURES, pressures),
                    ],
                ),
                bind(
                    Kernel::ElasticElementForces,
                    elements,
                    &[
                        (ELEMENTS, &element_buffer),
                        (DISPLACEMENTS, displacements),
                        (DILATIONS, dilations),
                        (PRESSURES, pressures),
                        (ELEMENT_FORCES, element_forces),
                    ],
                ),
                gather(&bind, &offsets, &entries, nodes, element_forces, forces),
            ]
        };
        let viscous_phases = |displacements: &wgpu::Buffer,
                              velocities: &wgpu::Buffer,
                              element_forces: &wgpu::Buffer,
                              forces: &wgpu::Buffer| {
            [
                bind(
                    Kernel::ViscousElementForces,
                    elements,
                    &[
                        (ELEMENTS, &element_buffer),
                        (DISPLACEMENTS, displacements),
                        (VELOCITIES, velocities),
                        (ELEMENT_FORCES, element_forces),
                    ],
                ),
                gather(&bind, &offsets, &entries, nodes, element_forces, forces),
            ]
        };
        let reduce =
            |items, reduction: &wgpu::Buffer, input: &wgpu::Buffer, output: &wgpu::Buffer| {
                bind(
                    Kernel::ReducePartials,
                    items,
                    &[(REDUCTION, reduction), (TERMS, input), (PARTIALS, output)],
                )
            };
        let reduce_row = |reduction: &wgpu::Buffer, input: &wgpu::Buffer, rows: &wgpu::Buffer| {
            bind(
                Kernel::ReduceRow,
                1,
                &[(REDUCTION, reduction), (PARTIALS, input), (LOG_ROWS, rows)],
            )
        };

        // The step.
        let [dilate, volume] =
            volume_phases(&displacements, &dilations, &volume_changes, &counters);
        let [pressure, elastic, gather_elastic] = force_phases(
            &displacements,
            &dilations,
            &volume_changes,
            &pressures,
            &element_forces,
            &elastic_forces,
        );
        let mut element_phase = vec![elastic];
        let mut gather_phase = vec![gather_elastic];
        if viscous {
            let [viscous_elements, gather_viscous] = viscous_phases(
                &displacements,
                &velocities,
                &element_viscous_forces,
                &viscous_forces,
            );
            element_phase.push(viscous_elements);
            gather_phase.push(gather_viscous);
        }
        let contact_phase = vec![
            bind(
                Kernel::Contact,
                surface_count,
                &[
                    (NODES, &node_buffer),
                    (SURFACE_NODES, &surface_buffer),
                    (GRID_VALUES, &grid_values),
                    (FINE_MAP, &fine_map),
                    (FINE_VALUES, &fine_values),
                    (DISPLACEMENTS, &displacements),
                    (VELOCITIES, &velocities),
                    (ELASTIC_FORCES, &elastic_forces),
                    (VISCOUS_FORCES, &viscous_forces),
                    (CONTACTS, &contacts),
                    (ANCHORS, &anchor_buffer),
                    (COUNTERS, &counters),
                    (TERMS, &contact_terms),
                    (MAXIMA, &maxima),
                ],
            ),
            reduce(
                surface_count,
                &contact_reduction,
                &contact_terms,
                &contact_partials,
            ),
            reduce_row(&contact_reduction, &contact_partials, &contact_rows.buffer),
        ];
        let integrate_phase = vec![bind(
            Kernel::Integrate,
            nodes,
            &[
                (NODES, &node_buffer),
                (DISPLACEMENTS, &displacements),
                (VELOCITIES, &velocities),
                (PREVIOUS_VELOCITIES, &previous_velocities),
                (ELASTIC_FORCES, &elastic_forces),
                (VISCOUS_FORCES, &viscous_forces),
                (CONTACTS, &contacts),
            ],
        )];
        let boundary_phase = vec![
            bind(
                Kernel::BoundaryConditions,
                nodes,
                &[
                    (NODES, &node_buffer),
                    (DISPLACEMENTS, &displacements),
                    (VELOCITIES, &velocities),
                    (PREVIOUS_VELOCITIES, &previous_velocities),
                    (VISCOUS_FORCES, &viscous_forces),
                    (CONTACTS, &contacts),
                    (TERMS, &boundary_terms),
                ],
            ),
            reduce(
                nodes,
                &boundary_reduction,
                &boundary_terms,
                &boundary_partials,
            ),
            reduce_row(
                &boundary_reduction,
                &boundary_partials,
                &boundary_rows.buffer,
            ),
        ];
        let accumulate_phase = vec![bind(
            Kernel::Accumulate,
            nodes,
            &[
                (NODES, &node_buffer),
                (DISPLACEMENTS, &displacements),
                (CONTACTS, &contacts),
                (WINDOW, &window),
            ],
        )];

        // The monitors' read: phases 1–2 on scratch arrays, uncounted.
        let mut read = volume_phases(
            &displacements,
            &scratch_dilations,
            &scratch_volume_changes,
            &scratch_counters,
        )
        .to_vec();
        read.extend([
            bind(
                Kernel::ElementEnergies,
                elements,
                &[
                    (ELEMENTS, &element_buffer),
                    (DISPLACEMENTS, &displacements),
                    (DILATIONS, &scratch_dilations),
                    (TERMS, &element_energy_terms),
                ],
            ),
            bind(
                Kernel::NodeEnergies,
                nodes,
                &[
                    (NODES, &node_buffer),
                    (VELOCITIES, &velocities),
                    (VOLUME_CHANGES, &scratch_volume_changes),
                    (CONTACTS, &contacts),
                    (TERMS, &node_energy_terms),
                ],
            ),
            reduce(
                elements,
                &element_energy_reduction,
                &element_energy_terms,
                &element_energy_partials,
            ),
            reduce(
                nodes,
                &node_energy_reduction,
                &node_energy_terms,
                &node_energy_partials,
            ),
            reduce(surface_count, &maxima_reduction, &maxima, &maxima_partials),
        ]);

        // The estimate, on scratch arrays.
        let scratch = |displaced: &wgpu::Buffer, forces: &wgpu::Buffer| {
            let mut phases = volume_phases(
                displaced,
                &scratch_dilations,
                &scratch_volume_changes,
                &scratch_counters,
            )
            .to_vec();
            phases.extend(force_phases(
                displaced,
                &scratch_dilations,
                &scratch_volume_changes,
                &scratch_pressures,
                &scratch_element_forces,
                forces,
            ));
            phases
        };
        let mut estimate_start = vec![bind(
            Kernel::EstimateStart,
            nodes,
            &[
                (SCALARS, &scalars),
                (START_VECTOR, &start_buffer),
                (VECTOR, &vector),
                (MEASURED, &measured),
                (MAGNITUDES, &magnitudes),
            ],
        )];
        estimate_start.extend(scratch(&displacements, &base_forces));
        let largest = |reduction: &wgpu::Buffer| {
            [
                reduce(nodes, reduction, &magnitudes, &magnitude_partials),
                reduce_row(reduction, &magnitude_partials, &scalars),
            ]
        };
        let mut iteration_head = largest(&largest_reduction).to_vec();
        iteration_head.push(bind(Kernel::EstimateScale, 1, &[(SCALARS, &scalars)]));
        iteration_head.push(bind(
            Kernel::EstimateShift,
            nodes,
            &[
                (SCALARS, &scalars),
                (DISPLACEMENTS, &displacements),
                (VECTOR, &vector),
                (SHIFTED, &shifted),
            ],
        ));
        iteration_head.extend(scratch(&shifted, &shifted_forces));
        iteration_head.push(bind(
            Kernel::EstimateStiffness,
            nodes,
            &[
                (NODES, &node_buffer),
                (SCALARS, &scalars),
                (VECTOR, &vector),
                (NEXT_VECTOR, &next_vector),
                (BASE_FORCES, &base_forces),
                (SHIFTED_FORCES, &shifted_forces),
                (QUOTIENTS, &quotients),
            ],
        ));
        iteration_head.push(reduce(
            nodes,
            &quotient_reduction,
            &quotients,
            &quotient_partials,
        ));
        let viscous_at_vector = viscous_phases(
            &displacements,
            &vector,
            &scratch_element_forces,
            &viscous_at,
        )
        .to_vec();
        let mut iteration_tail = vec![bind(
            Kernel::EstimateNext,
            nodes,
            &[
                (NODES, &node_buffer),
                (SCALARS, &scalars),
                (NEXT_VECTOR, &next_vector),
                (VISCOUS_AT, &viscous_at),
                (MAGNITUDES, &magnitudes),
            ],
        )];
        iteration_tail.extend(largest(&size_reduction));
        iteration_tail.push(bind(
            Kernel::EstimateAdvance,
            nodes,
            &[
                (SCALARS, &scalars),
                (VECTOR, &vector),
                (MEASURED, &measured),
                (NEXT_VECTOR, &next_vector),
                (MAGNITUDES, &magnitudes),
            ],
        ));
        let mut estimate_damping = viscous_phases(
            &displacements,
            &measured,
            &scratch_element_forces,
            &viscous_at,
        )
        .to_vec();
        estimate_damping.push(bind(
            Kernel::EstimateDamping,
            nodes,
            &[
                (NODES, &node_buffer),
                (MEASURED, &measured),
                (VISCOUS_AT, &viscous_at),
                (DAMPINGS, &dampings),
            ],
        ));
        estimate_damping.push(reduce(
            nodes,
            &damping_reduction,
            &dampings,
            &damping_partials,
        ));

        let programs = Programs {
            phases: [
                vec![dilate],
                vec![volume],
                vec![pressure],
                element_phase,
                gather_phase,
                contact_phase,
                integrate_phase,
                boundary_phase,
                accumulate_phase,
            ],
            read,
            estimate_start,
            iteration_head,
            viscous_at_vector,
            iteration_tail,
            estimate_damping,
        };
        let buffers = Buffers {
            constants: constants_buffer,
            displacements,
            velocities,
            anchors: anchor_buffer,
            dilations,
            volume_changes,
            pressures,
            element_forces,
            element_viscous_forces,
            elastic_forces,
            viscous_forces,
            contacts,
            counters,
            window,
            element_energy_partials,
            node_energy_partials,
            maxima_partials,
            scalars,
            quotient_partials,
            damping_partials,
            contact_partials,
            contact_reduction,
            boundary_partials,
            boundary_reduction,
        };
        Ok(Self {
            device: device.clone(),
            recorder,
            kernels,
            buffers,
            programs,
            pending: Pending::default(),
            contact_rows,
            boundary_rows,
            motions: Vec::new(),
            totals: Totals::default(),
            window: Window {
                displacement_sums: vec![[0.0; 3]; nodes as usize],
                normal_force_sums: vec![0.0; surface_count as usize],
                friction_sums: vec![[0.0; 3]; surface_count as usize],
                steps: 0,
                unread: false,
            },
            host,
            tracks,
            node_count: nodes,
            element_count: elements,
            surface_count,
            shortest_edge: model.shortest_edge(),
            viscous,
            #[cfg(test)]
            pass_per_phase: false,
        })
    }

    /// Time every pass on the GPU from here on (recon §17d), on a context
    /// made with [`GpuContext::with_timestamps`]: see [`Self::pass_times`].
    ///
    /// # Panics
    ///
    /// When the context was not made that way.
    pub fn time_passes(&mut self) {
        self.record_pending();
        self.recorder.time_passes();
    }

    /// Each pass's time on the GPU, in seconds, since [`Self::time_passes`]
    /// or the last call, by label: `step` for the pending phases, one step or
    /// part of one; `read` for a monitor read's reduction; `estimate` for
    /// the step estimate. Waits for everything recorded.
    pub fn pass_times(&mut self) -> std::collections::BTreeMap<String, Vec<f64>> {
        self.record_pending();
        self.recorder.pass_times()
    }

    /// The host's time submitting since [`Self::time_passes`]: finishing each
    /// encoder and handing it to the queue.
    #[must_use]
    pub fn submit_times(&self) -> crate::submit::SubmitTimes {
        self.recorder.submit_times()
    }

    /// Append `phase` to the pending list, recording what is pending first
    /// when it must be; `carry` sets what the phase carries in the step's
    /// values.
    fn append(&mut self, phase: Phase, timing: Option<[f32; 2]>, carry: impl FnOnce(&mut Self)) {
        if self.pending.must_record_before(phase, timing) {
            self.record_pending();
        }
        carry(self);
        if let Some([dt, damping]) = timing {
            self.pending.values.dt = dt;
            self.pending.values.damping = damping;
            self.pending.timing = timing;
        }
        self.pending.phases.push(phase);
        #[cfg(test)]
        if self.pass_per_phase {
            self.record_pending();
        }
    }

    /// Record the pending phases as one pass, in one recorder step.
    fn record_pending(&mut self) {
        if self.pending.phases.is_empty() {
            return;
        }
        let pending = std::mem::take(&mut self.pending);
        let phases = &self.programs.phases;
        let dispatches = pending
            .phases
            .iter()
            .flat_map(|&phase| &phases[phase as usize]);
        record_pass(
            &mut self.recorder,
            &self.kernels,
            ("step", &pending.values),
            dispatches,
        );
    }

    /// The next row of `rows`, the log grown when it is full: a larger buffer,
    /// the rows so far copied in, and the reduction that writes it bound to it.
    fn next_row(&mut self, contact: bool) -> u32 {
        let (rows, width, reduction, partials, phase) = if contact {
            (
                &mut self.contact_rows,
                CONTACT_ROW,
                &self.buffers.contact_reduction,
                &self.buffers.contact_partials,
                Phase::Contact,
            )
        } else {
            (
                &mut self.boundary_rows,
                BOUNDARY_ROW,
                &self.buffers.boundary_reduction,
                &self.buffers.boundary_partials,
                Phase::BoundaryConditions,
            )
        };
        if rows.written == rows.capacity {
            let row_bytes = (width * std::mem::size_of::<f32>()) as u64;
            let capacity = rows.capacity * 2;
            let grown = self.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("log rows"),
                size: u64::from(capacity) * row_bytes,
                usage: Maker::USAGE,
                mapped_at_creation: false,
            });
            self.recorder.commands().copy_buffer_to_buffer(
                &rows.buffer,
                0,
                &grown,
                0,
                u64::from(rows.capacity) * row_bytes,
            );
            // Cannot fail: the recorder is built with `with_step_values`.
            let Some(step_values) = self.recorder.step_values_binding() else {
                unreachable!("a recorder built with step values binds them");
            };
            let shared_bindings = Shared {
                constants: &self.buffers.constants,
                step_values,
            };
            let row = self.kernels.bind(
                &self.device,
                &shared_bindings,
                Kernel::ReduceRow,
                1,
                &[
                    (REDUCTION, reduction),
                    (PARTIALS, partials),
                    (LOG_ROWS, &grown),
                ],
            );
            if let Some(last) = self.programs.phases[phase as usize].last_mut() {
                *last = row;
            }
            rows.buffer = grown;
            rows.capacity = capacity;
        }
        rows.written += 1;
        rows.written - 1
    }

    /// The log's rows written since the last `monitors` or `snapshot`, and the
    /// window sums if any step added to them, read with `more` in one read;
    /// the rows and sums taken into the host's totals, and `more`'s data
    /// returned.
    fn read_with_log(&mut self, more: &[(wgpu::Buffer, u64)]) -> Vec<Vec<u8>> {
        self.record_pending();
        let contact_bytes = bytes::<[f32; CONTACT_ROW]>(self.contact_rows.written);
        let boundary_bytes = bytes::<[f32; BOUNDARY_ROW]>(self.boundary_rows.written);
        let window_bytes = bytes::<f32>(3 * self.node_count + 4 * self.surface_count);
        let mut reads = vec![
            (&self.contact_rows.buffer, contact_bytes),
            (&self.boundary_rows.buffer, boundary_bytes),
        ];
        let window = self.window.unread;
        if window {
            reads.push((&self.buffers.window, window_bytes));
        }
        reads.extend(more.iter().map(|(buffer, size)| (buffer, *size)));
        let mut data = self.recorder.read_many(&reads);
        let rest = data.split_off(if window { 3 } else { 2 });
        self.totals
            .add_contact_rows(&values::<f32>(&data[0]), &self.motions, self.host.centroid);
        self.totals.add_boundary_rows(&values::<f32>(&data[1]));
        self.motions.clear();
        self.contact_rows.written = 0;
        self.boundary_rows.written = 0;
        if window {
            self.take_window(&values::<f32>(&data[2]));
        }
        rest
    }

    /// Add the device's window sums to the host's, and empty them.
    fn take_window(&mut self, sums: &[f32]) {
        let (nodes, surface) = (self.node_count as usize, self.surface_count as usize);
        for (sum, device) in self
            .window
            .displacement_sums
            .iter_mut()
            .zip(sums.chunks_exact(3))
        {
            for d in 0..3 {
                sum[d] += f64::from(device[d]);
            }
        }
        let normals = &sums[3 * nodes..3 * nodes + surface];
        for (sum, &device) in self.window.normal_force_sums.iter_mut().zip(normals) {
            *sum += f64::from(device);
        }
        let frictions = &sums[3 * nodes + surface..];
        for (sum, device) in self
            .window
            .friction_sums
            .iter_mut()
            .zip(frictions.chunks_exact(3))
        {
            for d in 0..3 {
                sum[d] += f64::from(device[d]);
            }
        }
        self.recorder
            .commands()
            .clear_buffer(&self.buffers.window, 0, None);
        self.window.unread = false;
    }

    /// Set the inverted-element and coarse-correction counts, to count past
    /// 2³² in a test.
    #[cfg(test)]
    fn set_counts(&mut self, inverted: u64, coarse: u64) {
        self.record_pending();
        let words: [u32; 4] = bytemuck::cast([inverted, coarse]);
        self.recorder
            .write(&self.buffers.counters, 0, bytemuck::cast_slice(&words));
    }
}

/// Record `dispatches` as one pass labelled `label`, in one recorder step
/// carrying `values`.
fn record_pass<'a>(
    recorder: &mut Recorder<StepValues>,
    kernels: &Kernels,
    (label, values): (&str, &StepValues),
    dispatches: impl IntoIterator<Item = &'a Dispatch>,
) {
    let offset = recorder.begin_step(values);
    {
        let mut pass = recorder.pass(label);
        for dispatch in dispatches {
            kernels.record(&mut pass, dispatch, offset);
        }
    }
    recorder.end_step();
}

/// A gather of `element_forces` into `forces`, for `nodes` nodes.
fn gather(
    bind: &impl Fn(Kernel, u32, &[(u32, &wgpu::Buffer)]) -> Dispatch,
    offsets: &wgpu::Buffer,
    entries: &wgpu::Buffer,
    nodes: u32,
    element_forces: &wgpu::Buffer,
    forces: &wgpu::Buffer,
) -> Dispatch {
    bind(
        Kernel::GatherForces,
        nodes,
        &[
            (OFFSETS, offsets),
            (ENTRIES, entries),
            (ELEMENT_FORCES, element_forces),
            (NODE_FORCES, forces),
        ],
    )
}

/// The f64 sum of each component's partials, `components` a partial.
fn partial_sums(data: &[u8], components: usize) -> Vec<f64> {
    let partials = values::<f32>(data);
    (0..components)
        .map(|c| {
            partials
                .iter()
                .skip(c)
                .step_by(components)
                .map(|&p| f64::from(p))
                .sum()
        })
        .collect()
}

/// The largest of each component's partials, `components` a partial, from 0.
fn partial_maxima(data: &[u8], components: usize) -> Vec<f64> {
    let partials = values::<f32>(data);
    (0..components)
        .map(|c| {
            partials
                .iter()
                .skip(c)
                .step_by(components)
                .fold(0.0, |m: f64, &p| m.max(f64::from(p)))
        })
        .collect()
}

impl Executor for GpuExecutor {
    fn node_count(&self) -> usize {
        self.node_count as usize
    }

    fn shortest_edge(&self) -> f64 {
        self.shortest_edge
    }

    fn epsilon(&self) -> f64 {
        f64::from(f32::EPSILON)
    }

    fn set_state(
        &mut self,
        time: f64,
        displacements: &[[f64; 3]],
        velocities: &[[f64; 3]],
        anchors: Option<&[[f64; 3]]>,
    ) {
        let nodes = self.node_count as usize;
        assert_eq!(displacements.len(), nodes, "one displacement per node");
        assert_eq!(velocities.len(), nodes, "one velocity per node");
        let host = &self.host;
        let projected = |values: &[[f64; 3]]| -> Vec<[f32; 3]> {
            (0..nodes)
                .map(|a| host.free_part(a, narrow3(values[a])))
                .collect()
        };
        let displacements = projected(displacements);
        let velocities = projected(velocities);
        let anchors: Vec<[f32; 3]> = if let Some(given) = anchors {
            assert_eq!(given.len(), nodes, "one anchor per node");
            host.surface_nodes
                .iter()
                .map(|&a| narrow3(given[a as usize]))
                .collect()
        } else {
            let pose = self.tracks.at(time);
            host.surface_nodes
                .iter()
                .map(|&a| {
                    let a = a as usize;
                    shared::pose_to_body(pose, shared::vec3_add(host.rest[a], displacements[a]))
                })
                .collect()
        };
        self.record_pending();
        let buffers = &self.buffers;
        for (buffer, data) in [
            (&buffers.displacements, &displacements),
            (&buffers.velocities, &velocities),
            (&buffers.anchors, &anchors),
        ] {
            self.recorder
                .write(buffer, 0, bytemuck::cast_slice(data.as_slice()));
        }
    }

    fn set_poses(
        &mut self,
        start: f64,
        interval: f64,
        poses: &[sim_soft_explicit::f64::Pose],
    ) -> Result<(), ObstacleError> {
        check_poses(start, interval, poses)?;
        self.tracks = Tracks::of(start, interval, poses);
        Ok(())
    }

    fn element_dilations(&mut self) {
        self.append(Phase::Dilations, None, |_| {});
    }

    fn gather_volume_changes(&mut self) {
        self.append(Phase::VolumeChanges, None, |_| {});
    }

    fn nodal_pressures(&mut self) {
        self.append(Phase::Pressures, None, |_| {});
    }

    fn element_forces(&mut self) {
        self.append(Phase::ElementForces, None, |_| {});
    }

    fn gather_forces(&mut self) {
        self.append(Phase::GatherForces, None, |_| {});
    }

    // f64 → f32 rounds, as the CPU executor at f32 narrows its boundary values.
    #[allow(clippy::cast_possible_truncation)]
    fn contact(&mut self, time: f64, dt: f64, damping: f64) {
        let (start, end) = (self.tracks.at(time), self.tracks.at(time + dt));
        let (from, to) = (self.tracks.exact.at(time), self.tracks.exact.at(time + dt));
        let (moved, turn) = rigid_motion(from, to);
        let motion = Motion {
            origin: [from.tx, from.ty, from.tz],
            moved,
            turn,
        };
        self.append(
            Phase::Contact,
            Some([dt as f32, damping as f32]),
            |executor| {
                let row = executor.next_row(true);
                let values = &mut executor.pending.values;
                values.contact_row = row;
                values.start = pose_values(start);
                values.end = pose_values(end);
                executor.motions.push(motion);
            },
        );
    }

    // f64 → f32 rounds, as the CPU executor at f32 narrows its boundary values.
    #[allow(clippy::cast_possible_truncation)]
    fn integrate(&mut self, dt: f64, damping: f64) {
        self.append(Phase::Integrate, Some([dt as f32, damping as f32]), |_| {});
    }

    // f64 → f32 rounds, as the CPU executor at f32 narrows its boundary values.
    #[allow(clippy::cast_possible_truncation)]
    fn boundary_conditions(&mut self, dt: f64, damping: f64) {
        self.append(
            Phase::BoundaryConditions,
            Some([dt as f32, damping as f32]),
            |executor| {
                executor.pending.values.boundary_row = executor.next_row(false);
            },
        );
    }

    fn accumulate(&mut self) {
        self.append(Phase::Accumulate, None, |executor| {
            executor.window.steps += 1;
            executor.window.unread = true;
        });
    }

    fn clear_accumulators(&mut self) {
        self.record_pending();
        self.recorder
            .commands()
            .clear_buffer(&self.buffers.window, 0, None);
        let window = &mut self.window;
        window.displacement_sums.fill([0.0; 3]);
        window.normal_force_sums.fill(0.0);
        window.friction_sums.fill([0.0; 3]);
        window.steps = 0;
        window.unread = false;
    }

    // Step counts stay far below 2^52, where u64 → f64 starts to round.
    #[allow(clippy::cast_precision_loss)]
    fn monitors(&mut self) -> Monitors {
        self.record_pending();
        record_pass(
            &mut self.recorder,
            &self.kernels,
            ("read", &StepValues::default()),
            &self.programs.read,
        );
        let (nodes, elements, surface) = (self.node_count, self.element_count, self.surface_count);
        let buffers = &self.buffers;
        let data = self.read_with_log(&[
            (
                buffers.element_energy_partials.clone(),
                bytes::<f32>(blocks(elements)),
            ),
            (
                buffers.node_energy_partials.clone(),
                bytes::<[f32; 3]>(blocks(nodes)),
            ),
            (
                buffers.maxima_partials.clone(),
                bytes::<[f32; 2]>(blocks(surface)),
            ),
            (buffers.counters.clone(), bytes::<u32>(4)),
        ]);
        let element_energy = partial_sums(&data[0], 1)[0];
        let node_sums = partial_sums(&data[1], 3);
        let maxima = partial_maxima(&data[2], 2);
        let counts: [u64; 2] = {
            let words = values::<u32>(&data[3]);
            [
                u64::from(words[0]) | (u64::from(words[1]) << 32),
                u64::from(words[2]) | (u64::from(words[3]) << 32),
            ]
        };
        let totals = self.totals;
        let mean = if totals.steps == 0 {
            0.0
        } else {
            1.0 / totals.steps as f64
        };
        self.totals.restart();
        Monitors {
            kinetic_energy: node_sums[0],
            internal_energy: element_energy + node_sums[2],
            contact_kinetic_energy: node_sums[1],
            contact_force: totals.resultant.map(|f| f * mean),
            contact_moment: totals.moment.map(|m| m * mean),
            normal_force: totals.normal * mean,
            steps: totals.steps,
            inverted_element_steps: counts[0],
            contact_work: totals.contact_work,
            obstacle_work: totals.obstacle_work,
            damping_loss: totals.damping_loss,
            max_penetration: maxima[0],
            deepest_prediction: maxima[1],
            coarse_corrections: counts[1],
        }
    }

    fn snapshot(&mut self) -> Snapshot {
        let (nodes, surface) = (self.node_count, self.surface_count);
        let buffers = &self.buffers;
        let data = self.read_with_log(&[
            (buffers.displacements.clone(), bytes::<[f32; 3]>(nodes)),
            (buffers.velocities.clone(), bytes::<[f32; 3]>(nodes)),
            (buffers.anchors.clone(), bytes::<[f32; 3]>(surface)),
        ]);
        let vectors = |data: &[u8]| -> Vec<[f64; 3]> {
            values::<[f32; 3]>(data).into_iter().map(widen3).collect()
        };
        let nodes = nodes as usize;
        let mut anchors = vec![[0.0; 3]; nodes];
        let mut normal_force_sums = vec![0.0; nodes];
        let mut friction_sums = vec![[0.0; 3]; nodes];
        let surface_anchors = vectors(&data[2]);
        for (i, &a) in self.host.surface_nodes.iter().enumerate() {
            anchors[a as usize] = surface_anchors[i];
            normal_force_sums[a as usize] = self.window.normal_force_sums[i];
            friction_sums[a as usize] = self.window.friction_sums[i];
        }
        Snapshot {
            displacements: vectors(&data[0]),
            velocities: vectors(&data[1]),
            anchors,
            displacement_sums: self.window.displacement_sums.clone(),
            normal_force_sums,
            friction_sums,
            accumulated_steps: self.window.steps,
        }
    }

    fn phase_outputs(&mut self) -> PhaseOutputs {
        self.record_pending();
        let (nodes, elements, surface) = (self.node_count, self.element_count, self.surface_count);
        let buffers = &self.buffers;
        let data = self.recorder.read_many(&[
            (&buffers.dilations, bytes::<f32>(elements)),
            (&buffers.volume_changes, bytes::<f32>(nodes)),
            (&buffers.pressures, bytes::<f32>(nodes)),
            (&buffers.element_forces, bytes::<[f32; 12]>(elements)),
            (
                &buffers.element_viscous_forces,
                bytes::<[f32; 12]>(elements),
            ),
            (&buffers.elastic_forces, bytes::<[f32; 3]>(nodes)),
            (&buffers.viscous_forces, bytes::<[f32; 3]>(nodes)),
            (&buffers.contacts, bytes::<[f32; 7]>(surface)),
        ]);
        let scalars =
            |data: &[u8]| -> Vec<f64> { values::<f32>(data).into_iter().map(f64::from).collect() };
        let vectors = |data: &[u8]| -> Vec<[f64; 3]> {
            values::<[f32; 3]>(data).into_iter().map(widen3).collect()
        };
        let twelves = |data: &[u8]| -> Vec<[f64; 12]> {
            values::<[f32; 12]>(data)
                .into_iter()
                .map(|f| f.map(f64::from))
                .collect()
        };
        let nodes = nodes as usize;
        let mut contact_forces = vec![[0.0; 3]; nodes];
        let mut normal_forces = vec![0.0; nodes];
        for (i, contact) in values::<[f32; 7]>(&data[7]).into_iter().enumerate() {
            let a = self.host.surface_nodes[i] as usize;
            contact_forces[a] = widen3([contact[0], contact[1], contact[2]]);
            normal_forces[a] = f64::from(contact[3]);
        }
        PhaseOutputs {
            dilations: scalars(&data[0]),
            volume_changes: scalars(&data[1]),
            pressures: scalars(&data[2]),
            element_forces: twelves(&data[3]),
            element_viscous_forces: twelves(&data[4]),
            elastic_forces: vectors(&data[5]),
            viscous_forces: vectors(&data[6]),
            contact_forces,
            normal_forces,
        }
    }

    // f64 → f32 rounds, as the CPU executor at f32 narrows its boundary values.
    #[allow(clippy::cast_possible_truncation)]
    fn estimate_top_mode(
        &mut self,
        iterations: usize,
        perturbation: f64,
        viscous_weight: f64,
    ) -> TopMode {
        self.record_pending();
        let weight = if self.viscous {
            viscous_weight as f32
        } else {
            0.0
        };
        let carried = StepValues {
            perturbation: perturbation as f32,
            weight,
            ..StepValues::default()
        };
        let programs = &self.programs;
        let mut dispatches: Vec<&[Dispatch]> = vec![&programs.estimate_start];
        for _ in 0..iterations {
            dispatches.push(&programs.iteration_head);
            if weight > 0.0 {
                dispatches.push(&programs.viscous_at_vector);
            }
            dispatches.push(&programs.iteration_tail);
        }
        if self.viscous {
            dispatches.push(&programs.estimate_damping);
        }
        record_pass(
            &mut self.recorder,
            &self.kernels,
            ("estimate", &carried),
            dispatches.into_iter().flatten(),
        );
        let nodes = self.node_count;
        let buffers = &self.buffers;
        let data = self.recorder.read_many(&[
            (&buffers.scalars, bytes::<f32>(SCALARS_LEN as u32)),
            (&buffers.quotient_partials, bytes::<[f32; 2]>(blocks(nodes))),
            (&buffers.damping_partials, bytes::<[f32; 2]>(blocks(nodes))),
        ]);
        let taken = values::<f32>(&data[0])[TAKEN] != 0.0;
        let quotient = |data: &[u8]| {
            let sums = partial_sums(data, 2);
            sums[0] / sums[1]
        };
        TopMode {
            omega_squared: if taken { quotient(&data[1]) } else { 0.0 },
            damping_quotient: if self.viscous {
                quotient(&data[2])
            } else {
                0.0
            },
        }
    }
}
