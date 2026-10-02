//! The soft executor's entry points: each one's bindings, its pipeline, and
//! its dispatch (`soft.wgsl`).
//!
//! Every binding sits in group 0 under its own number, and an entry point's
//! layout lists only the ones it uses, so none binds more than the 16 storage
//! buffers a stage is given (`context.rs`). A [`Dispatch`] is an entry point
//! over one bind group: the same entry point runs on the step's arrays or on
//! scratch ones by the group it is given.

use crate::context::GpuContext;
use crate::submit::Recorder;
use crate::wgpu_helpers::{buf_entry, create_pipeline, storage_entry, uniform_entry};

use super::StepValues;

/// The binding numbers `soft.wgsl` declares.
pub(super) mod binding {
    pub const CONSTANTS: u32 = 0;
    pub const STEP_VALUES: u32 = 1;
    pub const NODES: u32 = 2;
    pub const ELEMENTS: u32 = 3;
    pub const OFFSETS: u32 = 4;
    pub const ENTRIES: u32 = 5;
    pub const SURFACE_NODES: u32 = 6;
    pub const GRID_VALUES: u32 = 7;
    pub const FINE_MAP: u32 = 8;
    pub const FINE_VALUES: u32 = 9;
    pub const DISPLACEMENTS: u32 = 10;
    pub const VELOCITIES: u32 = 11;
    pub const PREVIOUS_VELOCITIES: u32 = 12;
    pub const DILATIONS: u32 = 13;
    pub const VOLUME_CHANGES: u32 = 14;
    pub const PRESSURES: u32 = 15;
    pub const ELEMENT_FORCES: u32 = 16;
    pub const NODE_FORCES: u32 = 17;
    pub const ELASTIC_FORCES: u32 = 18;
    pub const VISCOUS_FORCES: u32 = 19;
    pub const CONTACTS: u32 = 20;
    pub const ANCHORS: u32 = 21;
    pub const COUNTERS: u32 = 22;
    pub const TERMS: u32 = 23;
    pub const PARTIALS: u32 = 24;
    pub const LOG_ROWS: u32 = 25;
    pub const REDUCTION: u32 = 26;
    pub const WINDOW: u32 = 27;
    pub const MAXIMA: u32 = 28;
    pub const SCALARS: u32 = 29;
    pub const START_VECTOR: u32 = 30;
    pub const VECTOR: u32 = 31;
    pub const MEASURED: u32 = 32;
    pub const NEXT_VECTOR: u32 = 33;
    pub const BASE_FORCES: u32 = 34;
    pub const SHIFTED: u32 = 35;
    pub const SHIFTED_FORCES: u32 = 36;
    pub const VISCOUS_AT: u32 = 37;
    pub const MAGNITUDES: u32 = 38;
    pub const QUOTIENTS: u32 = 39;
    pub const DAMPINGS: u32 = 40;
}

use binding::{
    ANCHORS, BASE_FORCES, CONSTANTS, CONTACTS, COUNTERS, DAMPINGS, DILATIONS, DISPLACEMENTS,
    ELASTIC_FORCES, ELEMENT_FORCES, ELEMENTS, ENTRIES, FINE_MAP, FINE_VALUES, GRID_VALUES,
    LOG_ROWS, MAGNITUDES, MAXIMA, MEASURED, NEXT_VECTOR, NODE_FORCES, NODES, OFFSETS, PARTIALS,
    PRESSURES, PREVIOUS_VELOCITIES, QUOTIENTS, REDUCTION, SCALARS, SHIFTED, SHIFTED_FORCES,
    START_VECTOR, STEP_VALUES, SURFACE_NODES, TERMS, VECTOR, VELOCITIES, VISCOUS_AT,
    VISCOUS_FORCES, VOLUME_CHANGES, WINDOW,
};

/// How `soft.wgsl` declares a binding.
#[derive(Clone, Copy)]
enum Kind {
    Uniform,
    StepValues,
    Read,
    ReadWrite,
}

const fn kind(binding: u32) -> Kind {
    match binding {
        CONSTANTS | REDUCTION => Kind::Uniform,
        STEP_VALUES => Kind::StepValues,
        NODES | ELEMENTS | OFFSETS | ENTRIES | SURFACE_NODES | GRID_VALUES | FINE_MAP
        | FINE_VALUES | START_VECTOR | BASE_FORCES | SHIFTED_FORCES | VISCOUS_AT => Kind::Read,
        _ => Kind::ReadWrite,
    }
}

/// An entry point of `soft.wgsl`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Kernel {
    ElementDilations,
    GatherVolumeChanges,
    NodalPressures,
    ElasticElementForces,
    ViscousElementForces,
    GatherForces,
    Contact,
    Integrate,
    BoundaryConditions,
    Accumulate,
    ReducePartials,
    ReduceRow,
    ElementEnergies,
    NodeEnergies,
    EstimateStart,
    EstimateScale,
    EstimateShift,
    EstimateStiffness,
    EstimateNext,
    EstimateAdvance,
    EstimateDamping,
}

impl Kernel {
    const ALL: [Self; 21] = [
        Self::ElementDilations,
        Self::GatherVolumeChanges,
        Self::NodalPressures,
        Self::ElasticElementForces,
        Self::ViscousElementForces,
        Self::GatherForces,
        Self::Contact,
        Self::Integrate,
        Self::BoundaryConditions,
        Self::Accumulate,
        Self::ReducePartials,
        Self::ReduceRow,
        Self::ElementEnergies,
        Self::NodeEnergies,
        Self::EstimateStart,
        Self::EstimateScale,
        Self::EstimateShift,
        Self::EstimateStiffness,
        Self::EstimateNext,
        Self::EstimateAdvance,
        Self::EstimateDamping,
    ];

    /// Its entry point in `soft.wgsl`.
    const fn entry(self) -> &'static str {
        match self {
            Self::ElementDilations => "element_dilations",
            Self::GatherVolumeChanges => "gather_volume_changes",
            Self::NodalPressures => "nodal_pressures",
            Self::ElasticElementForces => "elastic_element_forces",
            Self::ViscousElementForces => "viscous_element_forces",
            Self::GatherForces => "gather_forces",
            Self::Contact => "contact",
            Self::Integrate => "integrate",
            Self::BoundaryConditions => "boundary_conditions",
            Self::Accumulate => "accumulate",
            Self::ReducePartials => "reduce_partials",
            Self::ReduceRow => "reduce_row",
            Self::ElementEnergies => "element_energies",
            Self::NodeEnergies => "node_energies",
            Self::EstimateStart => "estimate_start",
            Self::EstimateScale => "estimate_scale",
            Self::EstimateShift => "estimate_shift",
            Self::EstimateStiffness => "estimate_stiffness",
            Self::EstimateNext => "estimate_next",
            Self::EstimateAdvance => "estimate_advance",
            Self::EstimateDamping => "estimate_damping",
        }
    }

    /// The bindings it uses besides the constants and the step values, which
    /// every entry point is given.
    const fn bindings(self) -> &'static [u32] {
        match self {
            Self::ElementDilations => &[ELEMENTS, DISPLACEMENTS, DILATIONS, COUNTERS],
            Self::GatherVolumeChanges => &[ELEMENTS, OFFSETS, ENTRIES, DILATIONS, VOLUME_CHANGES],
            Self::NodalPressures => &[NODES, VOLUME_CHANGES, PRESSURES],
            Self::ElasticElementForces => &[
                ELEMENTS,
                DISPLACEMENTS,
                DILATIONS,
                PRESSURES,
                ELEMENT_FORCES,
            ],
            Self::ViscousElementForces => &[ELEMENTS, DISPLACEMENTS, VELOCITIES, ELEMENT_FORCES],
            Self::GatherForces => &[OFFSETS, ENTRIES, ELEMENT_FORCES, NODE_FORCES],
            Self::Contact => &[
                NODES,
                SURFACE_NODES,
                GRID_VALUES,
                FINE_MAP,
                FINE_VALUES,
                DISPLACEMENTS,
                VELOCITIES,
                ELASTIC_FORCES,
                VISCOUS_FORCES,
                CONTACTS,
                ANCHORS,
                COUNTERS,
                TERMS,
                MAXIMA,
            ],
            Self::Integrate => &[
                NODES,
                DISPLACEMENTS,
                VELOCITIES,
                PREVIOUS_VELOCITIES,
                ELASTIC_FORCES,
                VISCOUS_FORCES,
                CONTACTS,
            ],
            Self::BoundaryConditions => &[
                NODES,
                DISPLACEMENTS,
                VELOCITIES,
                PREVIOUS_VELOCITIES,
                VISCOUS_FORCES,
                CONTACTS,
                TERMS,
            ],
            Self::Accumulate => &[NODES, DISPLACEMENTS, CONTACTS, WINDOW],
            Self::ReducePartials => &[REDUCTION, TERMS, PARTIALS],
            Self::ReduceRow => &[REDUCTION, PARTIALS, LOG_ROWS],
            Self::ElementEnergies => &[ELEMENTS, DISPLACEMENTS, DILATIONS, TERMS],
            Self::NodeEnergies => &[NODES, VELOCITIES, VOLUME_CHANGES, CONTACTS, TERMS],
            Self::EstimateStart => &[SCALARS, START_VECTOR, VECTOR, MEASURED, MAGNITUDES],
            Self::EstimateScale => &[SCALARS],
            Self::EstimateShift => &[SCALARS, DISPLACEMENTS, VECTOR, SHIFTED],
            Self::EstimateStiffness => &[
                NODES,
                SCALARS,
                VECTOR,
                NEXT_VECTOR,
                BASE_FORCES,
                SHIFTED_FORCES,
                QUOTIENTS,
            ],
            Self::EstimateNext => &[NODES, SCALARS, NEXT_VECTOR, VISCOUS_AT, MAGNITUDES],
            Self::EstimateAdvance => &[SCALARS, VECTOR, MEASURED, NEXT_VECTOR, MAGNITUDES],
            Self::EstimateDamping => &[NODES, MEASURED, VISCOUS_AT, DAMPINGS],
        }
    }

    /// Invocations a workgroup; a kernel of one workgroup runs one.
    const fn workgroup(self) -> Workgroups {
        match self {
            Self::ReducePartials => Workgroups::PerItems(super::TREE),
            Self::ReduceRow | Self::EstimateScale => Workgroups::One,
            _ => Workgroups::PerItems(WORKGROUP),
        }
    }
}

/// Invocations a workgroup for per-item work (`soft.wgsl`'s `WORKGROUP`).
const WORKGROUP: u32 = 64;

/// The most workgroups a dispatch dimension holds.
const DIMENSION: u32 = 65_535;

/// How many workgroups a kernel runs.
#[derive(Clone, Copy)]
enum Workgroups {
    /// One per this many items.
    PerItems(u32),
    /// One, whatever the items.
    One,
}

/// Every entry point's pipeline and layout.
pub(super) struct Kernels {
    pipelines: Vec<(wgpu::ComputePipeline, wgpu::BindGroupLayout)>,
}

impl Kernels {
    /// Compile `soft.wgsl` after the shared math, and build each entry
    /// point's layout and pipeline.
    pub(super) fn new(ctx: &GpuContext) -> Self {
        let source = format!(
            "{}\n{}",
            sim_soft_explicit::SHARED_WGSL,
            include_str!("soft.wgsl")
        );
        let module = ctx
            .device
            .create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("soft"),
                source: wgpu::ShaderSource::Wgsl(source.into()),
            });
        let pipelines = Kernel::ALL
            .iter()
            .map(|&kernel| {
                let entries: Vec<wgpu::BindGroupLayoutEntry> = [CONSTANTS, STEP_VALUES]
                    .iter()
                    .chain(kernel.bindings())
                    .map(|&b| match kind(b) {
                        Kind::Uniform => uniform_entry(b),
                        Kind::StepValues => Recorder::<StepValues>::step_values_layout_entry(b),
                        Kind::Read => storage_entry(b, true),
                        Kind::ReadWrite => storage_entry(b, false),
                    })
                    .collect();
                let layout =
                    ctx.device
                        .create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                            label: Some(kernel.entry()),
                            entries: &entries,
                        });
                let pipeline_layout =
                    ctx.device
                        .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                            label: Some(kernel.entry()),
                            bind_group_layouts: &[&layout],
                            push_constant_ranges: &[],
                        });
                let pipeline = create_pipeline(ctx, &pipeline_layout, &module, kernel.entry());
                (pipeline, layout)
            })
            .collect();
        Self { pipelines }
    }

    /// `kernel` over `buffers`, one per binding it uses besides the constants
    /// and the step values, which are `shared`'s.
    pub(super) fn bind(
        &self,
        device: &wgpu::Device,
        shared: &Shared<'_>,
        kernel: Kernel,
        items: u32,
        buffers: &[(u32, &wgpu::Buffer)],
    ) -> Dispatch {
        let mut entries = vec![
            buf_entry(CONSTANTS, shared.constants),
            wgpu::BindGroupEntry {
                binding: STEP_VALUES,
                resource: shared.step_values.clone(),
            },
        ];
        entries.extend(buffers.iter().map(|&(b, buffer)| buf_entry(b, buffer)));
        let group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some(kernel.entry()),
            layout: &self.pipelines[kernel as usize].1,
            entries: &entries,
        });
        Dispatch {
            kernel,
            group,
            items,
        }
    }

    /// Record `dispatch` into `pass`, its step values at `offset`.
    pub(super) fn record(
        &self,
        pass: &mut wgpu::ComputePass<'_>,
        dispatch: &Dispatch,
        offset: u32,
    ) {
        let groups = match dispatch.kernel.workgroup() {
            Workgroups::PerItems(size) => dispatch.items.div_ceil(size),
            Workgroups::One => 1,
        };
        if groups == 0 {
            return;
        }
        pass.set_pipeline(&self.pipelines[dispatch.kernel as usize].0);
        pass.set_bind_group(0, &dispatch.group, &[offset]);
        pass.dispatch_workgroups(groups.min(DIMENSION), groups.div_ceil(DIMENSION), 1);
    }
}

/// What every bind group shares: the constants, and the step values.
pub(super) struct Shared<'a> {
    pub constants: &'a wgpu::Buffer,
    pub step_values: wgpu::BindingResource<'a>,
}

/// An entry point over one bind group, for `items` elements, nodes or
/// surface nodes.
#[derive(Clone)]
pub(super) struct Dispatch {
    kernel: Kernel,
    group: wgpu::BindGroup,
    items: u32,
}

impl Dispatch {
    /// Its entry point in `soft.wgsl`.
    pub(super) const fn entry(&self) -> &'static str {
        self.kernel.entry()
    }
}
