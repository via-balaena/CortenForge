//! Recording GPU work, submitting it in bounded pieces, and reading it back.
//!
//! Two rules of wgpu 27 shape everything here (recon §17a):
//!
//! - **A command buffer holds a bounded number of compute passes on Metal.**
//!   wgpu-core opens two Metal command buffers for each pass, the queue holds
//!   4 096, and none is committed before the submit. Measured on an M4 Pro:
//!   2 047 passes in one command buffer complete; 2 048 block for good inside
//!   [`wgpu::CommandEncoder::finish`]. So a [`Recorder`] counts passes and
//!   submits well before that ([`PASS_CAP`]).
//! - **A `queue.write_buffer` runs before every command of the next submit.**
//!   A write issued while work is recorded but not submitted overtakes all of
//!   it. So each step's values go into a ring written at the submit, one slot
//!   a step, and a host write through [`Recorder::write`] first submits what
//!   was recorded before it.
//!
//! Reads, writes and submits happen between steps, and stop with a message
//! inside one.

use std::sync::mpsc;

use bytemuck::Pod;

use crate::context::GpuContext;

/// Compute passes one submit may hold.
///
/// A run of other commands (clears, copies) between two passes opens one more
/// Metal command buffer, so a submit at the cap holds at most about 3 072 of
/// the queue's 4 096.
pub const PASS_CAP: u32 = 1024;

/// Compute passes one step may open when its steps carry values.
///
/// A [`Recorder`] submits before a step and at its end once this many are
/// pending, so a step starts with fewer. A step's values sit in one submit's
/// ring, so a step carrying them cannot be split across submits: it may open
/// this many, and a submit then holds at most [`PASS_CAP`]. A step carrying no
/// values has no such limit; the recorder also submits inside it, at the cap.
pub const STEP_PASS_CAP: u32 = PASS_CAP / 2;

/// Something compute passes and commands are recorded into.
///
/// A [`Recorder`] counts and bounds what it records; a bare
/// [`wgpu::CommandEncoder`] does not, and suits a caller that records one
/// small piece and submits it itself.
pub trait Recording {
    /// Open a compute pass labelled `label`.
    fn pass(&mut self, label: &str) -> wgpu::ComputePass<'_>;

    /// The encoder, for commands outside a pass (clears, copies). A pass
    /// opened on it directly escapes a [`Recorder`]'s count: open passes with
    /// [`Self::pass`].
    fn commands(&mut self) -> &mut wgpu::CommandEncoder;
}

impl Recording for wgpu::CommandEncoder {
    fn pass(&mut self, label: &str) -> wgpu::ComputePass<'_> {
        self.begin_compute_pass(&wgpu::ComputePassDescriptor {
            label: Some(label),
            timestamp_writes: None,
        })
    }

    fn commands(&mut self) -> &mut wgpu::CommandEncoder {
        self
    }
}

/// Records GPU work, submits it within [`PASS_CAP`], and reads buffers back.
///
/// `T` is the value each step carries to its passes, bound by a dynamic offset
/// into a uniform ring; `()` for work whose steps carry none.
pub struct Recorder<T: Pod = ()> {
    device: wgpu::Device,
    queue: wgpu::Queue,
    encoder: Option<wgpu::CommandEncoder>,
    /// Compute passes in the pending encoder.
    passes: u32,
    /// Compute passes the open step has opened.
    step_passes: u32,
    /// Steps ended in the pending encoder, and so slots staged in the ring.
    steps: u32,
    in_step: bool,
    ring: Option<Ring>,
    values: std::marker::PhantomData<T>,
}

/// The uniform ring behind each step's values: one slot a step, each at the
/// device's `min_uniform_buffer_offset_alignment`, staged on the host and
/// written at the submit.
struct Ring {
    buffer: wgpu::Buffer,
    stride: u64,
    slots: u32,
    staged: Vec<u8>,
}

impl Recorder<()> {
    /// A recorder whose steps carry no values.
    #[must_use]
    pub fn new(ctx: &GpuContext) -> Self {
        Self::build(ctx, None)
    }
}

impl<T: Pod> Recorder<T> {
    /// A recorder whose steps each carry a `T`, with room for `slots` steps a
    /// submit.
    ///
    /// # Panics
    ///
    /// If `T` is zero-sized or `slots` is zero.
    #[must_use]
    #[allow(clippy::cast_possible_truncation)] // a ring of u32 slots, a few hundred KiB
    pub fn with_step_values(ctx: &GpuContext, slots: u32) -> Self {
        let size = std::mem::size_of::<T>() as u64;
        assert!(size > 0, "a step value must have a size");
        assert!(slots > 0, "the ring needs at least one slot");
        let align = u64::from(ctx.device.limits().min_uniform_buffer_offset_alignment);
        let stride = size.div_ceil(align) * align;
        let buffer = ctx.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("step_values"),
            size: stride * u64::from(slots),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let staged = vec![0; (stride * u64::from(slots)) as usize];
        Self::build(
            ctx,
            Some(Ring {
                buffer,
                stride,
                slots,
                staged,
            }),
        )
    }

    fn build(ctx: &GpuContext, ring: Option<Ring>) -> Self {
        Self {
            device: ctx.device.clone(),
            queue: ctx.queue.clone(),
            encoder: None,
            passes: 0,
            step_passes: 0,
            steps: 0,
            in_step: false,
            ring,
            values: std::marker::PhantomData,
        }
    }

    /// The bind-group layout entry for the step values at `binding`: a
    /// uniform with a dynamic offset.
    #[must_use]
    pub const fn step_values_layout_entry(binding: u32) -> wgpu::BindGroupLayoutEntry {
        wgpu::BindGroupLayoutEntry {
            binding,
            visibility: wgpu::ShaderStages::COMPUTE,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: true,
                min_binding_size: wgpu::BufferSize::new(std::mem::size_of::<T>() as u64),
            },
            count: None,
        }
    }

    /// The step values' binding, for a bind group built on
    /// [`Self::step_values_layout_entry`]; `None` when steps carry no values.
    #[must_use]
    pub fn step_values_binding(&self) -> Option<wgpu::BindingResource<'_>> {
        self.ring.as_ref().map(|ring| {
            wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                buffer: &ring.buffer,
                offset: 0,
                size: wgpu::BufferSize::new(std::mem::size_of::<T>() as u64),
            })
        })
    }

    /// Begin a step carrying `values`, and return the dynamic offset its
    /// passes bind the step values at (0 when steps carry none). Submits
    /// first once [`STEP_PASS_CAP`] passes are pending, as from passes opened
    /// outside steps, so the step has its whole budget.
    ///
    /// # Panics
    ///
    /// Inside a step.
    #[allow(clippy::cast_possible_truncation)] // an offset within a ring of u32 slots
    pub fn begin_step(&mut self, values: &T) -> u32 {
        assert!(!self.in_step, "begin_step inside a step");
        if self.passes >= STEP_PASS_CAP {
            self.submit();
        }
        self.in_step = true;
        self.step_passes = 0;
        let Some(ring) = self.ring.as_mut() else {
            return 0;
        };
        let at = u64::from(self.steps) * ring.stride;
        let bytes = bytemuck::bytes_of(values);
        let start = at as usize;
        ring.staged[start..start + bytes.len()].copy_from_slice(bytes);
        at as u32
    }

    /// End the step, and submit once half the pass cap is used or the ring
    /// is full.
    ///
    /// # Panics
    ///
    /// Outside a step.
    pub fn end_step(&mut self) {
        assert!(self.in_step, "end_step outside a step");
        self.in_step = false;
        self.steps += 1;
        let ring_full = self
            .ring
            .as_ref()
            .is_some_and(|ring| self.steps == ring.slots);
        if self.passes >= STEP_PASS_CAP || ring_full {
            self.submit();
        }
    }

    /// Submit everything recorded, the ring's staged slots written first.
    ///
    /// # Panics
    ///
    /// Inside a step.
    pub fn submit(&mut self) -> Option<wgpu::SubmissionIndex> {
        assert!(
            !self.in_step,
            "a submit inside a step: submit between steps"
        );
        self.flush()
    }

    /// Submit everything recorded, inside a step or not: only where no step's
    /// values can split, which [`Self::submit`] and the pass cap ensure.
    #[allow(clippy::cast_possible_truncation)] // within the ring, see `with_step_values`
    fn flush(&mut self) -> Option<wgpu::SubmissionIndex> {
        let steps = std::mem::take(&mut self.steps);
        self.passes = 0;
        // Steps that recorded nothing leave nothing to submit or bind.
        let encoder = self.encoder.take()?;
        if let Some(ring) = self.ring.as_ref() {
            let used = (u64::from(steps) * ring.stride) as usize;
            if used > 0 {
                self.queue
                    .write_buffer(&ring.buffer, 0, &ring.staged[..used]);
            }
        }
        Some(self.queue.submit([encoder.finish()]))
    }

    /// Write `data` into `buffer` at `offset`, after everything recorded
    /// before it.
    ///
    /// # Panics
    ///
    /// Inside a step.
    pub fn write(&mut self, buffer: &wgpu::Buffer, offset: u64, data: &[u8]) {
        assert!(!self.in_step, "a write inside a step would overtake it");
        self.submit();
        self.queue.write_buffer(buffer, offset, data);
    }

    /// Read `count` values of `P` from the start of `buffer`, after
    /// everything recorded before it.
    ///
    /// # Panics
    ///
    /// Inside a step, when `count` values of `P` are not a multiple of 4 bytes,
    /// or when the GPU wait or the buffer's mapping fails.
    pub fn read<P: Pod>(&mut self, buffer: &wgpu::Buffer, count: usize) -> Vec<P> {
        let bytes = (count * std::mem::size_of::<P>()) as u64;
        values(&self.read_many(&[(buffer, bytes)])[0])
    }

    /// Read the first `bytes` of each buffer named, in one submit and one
    /// wait, after everything recorded before them.
    ///
    /// # Panics
    ///
    /// Inside a step, when a byte count is not a multiple of 4 (wgpu copies
    /// buffers in 4-byte units), or when the GPU wait or a buffer's mapping
    /// fails. The wait has no timeout: it waits for everything submitted before it, whose
    /// length the caller sets, and [`PASS_CAP`] prevents the one hang measured.
    // Panicking is the contract: the executor trait's reads return values, not
    // errors, and a read that cannot finish leaves nothing to return.
    #[allow(clippy::panic)]
    pub fn read_many(&mut self, buffers: &[(&wgpu::Buffer, u64)]) -> Vec<Vec<u8>> {
        assert!(!self.in_step, "a read inside a step: read between steps");
        for &(_, bytes) in buffers {
            assert!(
                bytes % wgpu::COPY_BUFFER_ALIGNMENT == 0,
                "a read of {bytes} bytes: reads come in multiples of 4 bytes"
            );
        }
        // An empty read needs no staging: wgpu maps no empty slice.
        let staging: Vec<Option<wgpu::Buffer>> = buffers
            .iter()
            .map(|&(buffer, bytes)| {
                (bytes > 0).then(|| {
                    let copy = self.device.create_buffer(&wgpu::BufferDescriptor {
                        label: Some("read_staging"),
                        size: bytes,
                        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                        mapped_at_creation: false,
                    });
                    self.commands()
                        .copy_buffer_to_buffer(buffer, 0, &copy, 0, bytes);
                    copy
                })
            })
            .collect();
        let submitted = self.submit();

        let (sent, mapped) = mpsc::channel();
        for (i, copy) in staging.iter().enumerate() {
            if let Some(copy) = copy {
                let sent = sent.clone();
                copy.slice(..)
                    .map_async(wgpu::MapMode::Read, move |result| {
                        // The receiver outlives the wait, so a send cannot fail.
                        sent.send((i, result)).unwrap_or_default();
                    });
            }
        }
        drop(sent);
        if let Err(err) = self.device.poll(wgpu::PollType::Wait {
            submission_index: submitted,
            timeout: None,
        }) {
            panic!("the GPU wait for a read failed: {err}");
        }
        let expected = staging.iter().flatten().count();
        let mut seen = 0;
        for (i, result) in mapped.try_iter() {
            if let Err(err) = result {
                panic!("mapping read buffer {i} failed: {err}");
            }
            seen += 1;
        }
        assert_eq!(seen, expected, "read buffers still unmapped after the wait");
        staging
            .iter()
            .map(|copy| {
                copy.as_ref().map_or_else(Vec::new, |copy| {
                    let data = copy.slice(..).get_mapped_range().to_vec();
                    copy.unmap();
                    data
                })
            })
            .collect()
    }
}

impl<T: Pod> Recording for Recorder<T> {
    /// Submits itself once [`PASS_CAP`] passes are pending, where no step's
    /// values can split: outside a step, or in a step that carries none.
    ///
    /// # Panics
    ///
    /// When a step carrying values has opened [`STEP_PASS_CAP`] passes: it
    /// stops here rather than split its values or block in
    /// [`wgpu::CommandEncoder::finish`].
    fn pass(&mut self, label: &str) -> wgpu::ComputePass<'_> {
        let values_in_step = self.in_step && self.ring.is_some();
        if values_in_step {
            assert!(
                self.step_passes < STEP_PASS_CAP,
                "a step carrying values opened more than {STEP_PASS_CAP} compute \
                 passes, and its values cannot split across submits (recon §17a)"
            );
            self.step_passes += 1;
        }
        if self.passes == PASS_CAP {
            // `begin_step` and `end_step` keep a step carrying values under the
            // cap; reaching it inside one means that bound was broken.
            assert!(
                !values_in_step,
                "{PASS_CAP} compute passes pending inside a step carrying values"
            );
            self.flush();
        }
        self.passes += 1;
        self.commands().pass(label)
    }

    fn commands(&mut self) -> &mut wgpu::CommandEncoder {
        let device = &self.device;
        self.encoder.get_or_insert_with(|| {
            device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("recorder"),
            })
        })
    }
}

#[cfg(test)]
mod tests;

/// The values of `P` in `bytes` read back, whatever the bytes' alignment.
#[must_use]
pub fn values<P: Pod>(bytes: &[u8]) -> Vec<P> {
    bytes
        .chunks_exact(std::mem::size_of::<P>())
        .map(bytemuck::pod_read_unaligned)
        .collect()
}

/// Read `count` values of `P` from the start of `buffer`: a read with
/// nothing recorded before it.
///
/// # Panics
///
/// When `count` values of `P` are not a multiple of 4 bytes, or when the GPU
/// wait or the buffer's mapping fails.
#[must_use]
pub fn read_buffer<P: Pod>(ctx: &GpuContext, buffer: &wgpu::Buffer, count: usize) -> Vec<P> {
    Recorder::new(ctx).read(buffer, count)
}
