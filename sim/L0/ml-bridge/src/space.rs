//! Observation and action space definitions.
//!
//! [`ObservationSpace`] maps selected slices of [`Data`] to a flat
//! [`Tensor`].  [`ActionSpace`] maps a flat [`Tensor`] back into [`Data`]
//! fields.
//!
//! Both spaces are built via builder APIs that validate ranges against
//! the [`Model`] at `build()` time — invalid ranges produce [`SpaceError`],
//! never runtime panics during extraction/injection.

use std::ops::Range;

use sim_core::{Data, Model};

use crate::error::SpaceError;
use crate::tensor::{Tensor, TensorSpec};

// ─── Extractor ──────────────────────────────────────────────────────────────

/// A single extraction: "read these indices from this field of Data."
///
/// Flat fields use element indices.  Per-body fields use body indices and
/// flatten automatically (3 floats per body for xpos, 4 for xquat, 6 for
/// cvel).
#[derive(Debug, Clone)]
enum Extractor {
    // Flat fields — range is element indices into the DVector/Vec.
    Qpos(Range<usize>),
    Qvel(Range<usize>),
    Qacc(Range<usize>),
    Ctrl(Range<usize>),
    Sensordata(Range<usize>),
    ActuatorForce(Range<usize>),
    QfrcConstraint(Range<usize>),

    // Per-body fields — range is body indices, flattened on extraction.
    Xpos(Range<usize>),
    Xquat(Range<usize>),
    Cvel(Range<usize>),

    // Per-site fields — range is site indices, flattened on extraction.
    SiteXpos(Range<usize>),

    // Scalars.
    ContactCount,
    Time,
    Energy,
}

impl Extractor {
    /// Number of `f32` elements this extractor produces.
    fn dim(&self) -> usize {
        match self {
            Self::Qpos(r)
            | Self::Qvel(r)
            | Self::Qacc(r)
            | Self::Ctrl(r)
            | Self::Sensordata(r)
            | Self::ActuatorForce(r)
            | Self::QfrcConstraint(r) => r.len(),

            Self::Xpos(r) | Self::SiteXpos(r) => r.len() * 3,
            Self::Xquat(r) => r.len() * 4,
            Self::Cvel(r) => r.len() * 6,

            Self::ContactCount | Self::Time => 1,
            Self::Energy => 2,
        }
    }

    /// Extract values from `data` into `buf`, which must have length == `self.dim()`.
    #[allow(clippy::cast_possible_truncation)]
    fn extract(&self, data: &Data, buf: &mut [f32]) {
        match self {
            Self::Qpos(r) => copy_dvec(data.qpos.as_slice(), r, buf),
            Self::Qvel(r) => copy_dvec(data.qvel.as_slice(), r, buf),
            Self::Qacc(r) => copy_dvec(data.qacc.as_slice(), r, buf),
            Self::Ctrl(r) => copy_dvec(data.ctrl.as_slice(), r, buf),
            Self::Sensordata(r) => copy_dvec(data.sensordata.as_slice(), r, buf),
            Self::ActuatorForce(r) => copy_vec_f64(&data.actuator_force, r, buf),
            Self::QfrcConstraint(r) => copy_dvec(data.qfrc_constraint.as_slice(), r, buf),

            Self::Xpos(r) => {
                let mut offset = 0;
                for body in r.clone() {
                    let v = &data.xpos[body];
                    buf[offset] = v[0] as f32;
                    buf[offset + 1] = v[1] as f32;
                    buf[offset + 2] = v[2] as f32;
                    offset += 3;
                }
            }
            Self::SiteXpos(r) => {
                let mut offset = 0;
                for site in r.clone() {
                    let v = &data.site_xpos[site];
                    buf[offset] = v[0] as f32;
                    buf[offset + 1] = v[1] as f32;
                    buf[offset + 2] = v[2] as f32;
                    offset += 3;
                }
            }
            Self::Xquat(r) => {
                let mut offset = 0;
                for body in r.clone() {
                    let q = &data.xquat[body];
                    buf[offset] = q.w as f32;
                    buf[offset + 1] = q.i as f32;
                    buf[offset + 2] = q.j as f32;
                    buf[offset + 3] = q.k as f32;
                    offset += 4;
                }
            }
            Self::Cvel(r) => {
                let mut offset = 0;
                for body in r.clone() {
                    let sv = &data.cvel[body];
                    for i in 0..6 {
                        buf[offset + i] = sv[i] as f32;
                    }
                    offset += 6;
                }
            }

            Self::ContactCount => {
                #[allow(clippy::cast_precision_loss)]
                {
                    buf[0] = data.ncon as f32;
                }
            }
            Self::Time => {
                buf[0] = data.time as f32;
            }
            Self::Energy => {
                buf[0] = data.energy_kinetic as f32;
                buf[1] = data.energy_potential as f32;
            }
        }
    }
}

/// Copy a range from a flat `f64` slice (`DVector` backing) to `f32` buf.
#[allow(clippy::cast_possible_truncation)]
fn copy_dvec(src: &[f64], range: &Range<usize>, dst: &mut [f32]) {
    for (d, &s) in dst.iter_mut().zip(&src[range.clone()]) {
        *d = s as f32;
    }
}

/// Copy a range from a `Vec<f64>` to `f32` buf.
#[allow(clippy::cast_possible_truncation)]
fn copy_vec_f64(src: &[f64], range: &Range<usize>, dst: &mut [f32]) {
    for (d, &s) in dst.iter_mut().zip(&src[range.clone()]) {
        *d = s as f32;
    }
}

// ─── ObsSegment ────────────────────────────────────────────────────────────

/// One segment of the observation vector, describing where an extractor's
/// output lands in the flat obs buffer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ObsSegment {
    /// Human-readable label, e.g. `"qpos(0..2)"`, `"sensor(cart_pos)"`, `"energy"`.
    pub label: String,
    /// Start index in the flat observation vector.
    pub offset: usize,
    /// Number of `f32` elements this segment occupies.
    pub dim: usize,
}

// ─── ObservationSpace ───────────────────────────────────────────────────────

/// Selects which slices of `Data` become the observation vector.
///
/// The observation is built by concatenating selected slices in the order
/// they were added.  The total dimension is the sum of all selected slice
/// lengths.
///
/// Constructed via [`ObservationSpace::builder()`].
#[derive(Debug, Clone)]
#[cfg_attr(feature = "bevy", derive(bevy_ecs::prelude::Resource))]
pub struct ObservationSpace {
    extractors: Vec<Extractor>,
    segments: Vec<ObsSegment>,
    dim: usize,
}

impl ObservationSpace {
    /// Start building an observation space.
    #[must_use]
    pub const fn builder() -> ObservationSpaceBuilder {
        ObservationSpaceBuilder {
            entries: Vec::new(),
        }
    }

    /// Total observation dimension (number of `f32` elements).
    #[must_use]
    pub const fn dim(&self) -> usize {
        self.dim
    }

    /// The segments describing the layout of the observation vector.
    ///
    /// Each segment tells you which extractor produced which slice of the
    /// flat obs buffer — useful for labeling HUD displays.
    #[must_use]
    pub fn segments(&self) -> &[ObsSegment] {
        &self.segments
    }

    /// A [`TensorSpec`] describing this observation space (unbounded).
    #[must_use]
    pub fn spec(&self) -> TensorSpec {
        TensorSpec {
            shape: vec![self.dim],
            low: None,
            high: None,
        }
    }

    /// Extract an observation from a single `Data` instance.
    #[must_use]
    pub fn extract(&self, data: &Data) -> Tensor {
        let mut buf = vec![0.0_f32; self.dim];
        let mut offset = 0;
        for ext in &self.extractors {
            let n = ext.dim();
            ext.extract(data, &mut buf[offset..offset + n]);
            offset += n;
        }
        Tensor::from_slice(&buf, &[self.dim])
    }

    /// Extract observations from `N` `Data` instances into a batched tensor.
    ///
    /// Returns shape `[n, dim]`.  Writes directly into a contiguous buffer —
    /// no intermediate per-env tensors.
    #[must_use]
    pub fn extract_batch<'a>(&self, envs: impl ExactSizeIterator<Item = &'a Data>) -> Tensor {
        let n = envs.len();
        let mut buf = vec![0.0_f32; n * self.dim];
        for (i, data) in envs.enumerate() {
            let row_start = i * self.dim;
            let mut offset = row_start;
            for ext in &self.extractors {
                let d = ext.dim();
                ext.extract(data, &mut buf[offset..offset + d]);
                offset += d;
            }
        }
        Tensor::from_slice(&buf, &[n, self.dim])
    }
}

// ─── Builder ────────────────────────────────────────────────────────────────

/// A pending entry in the builder — either a resolved extractor or
/// a named sensor that needs resolution at `build()` time.
#[derive(Debug, Clone)]
enum BuilderEntry {
    /// Already resolved to a concrete extractor.
    Resolved(Extractor),
    /// A sensor looked up by name — resolved at `build()` time.
    SensorByName(String),
    /// Shorthand that expands at `build()` time using Model dimensions.
    AllQpos,
    /// Shorthand that expands at `build()` time using Model dimensions.
    AllQvel,
    /// Shorthand that expands at `build()` time using Model dimensions.
    AllSensordata,
}

/// Builder for [`ObservationSpace`].
///
/// Add extractors in the order you want them concatenated in the
/// observation vector.  Call [`build()`](Self::build) to validate all
/// ranges against the model and freeze the space.
#[derive(Debug, Clone)]
pub struct ObservationSpaceBuilder {
    entries: Vec<BuilderEntry>,
}

impl ObservationSpaceBuilder {
    // ── flat fields (element-index ranges) ───────────────────────────────

    /// Observe `data.qpos[range]`.
    #[must_use]
    pub fn qpos(mut self, range: Range<usize>) -> Self {
        self.entries
            .push(BuilderEntry::Resolved(Extractor::Qpos(range)));
        self
    }

    /// Observe `data.qvel[range]`.
    #[must_use]
    pub fn qvel(mut self, range: Range<usize>) -> Self {
        self.entries
            .push(BuilderEntry::Resolved(Extractor::Qvel(range)));
        self
    }

    /// Observe `data.qacc[range]`.
    #[must_use]
    pub fn qacc(mut self, range: Range<usize>) -> Self {
        self.entries
            .push(BuilderEntry::Resolved(Extractor::Qacc(range)));
        self
    }

    /// Observe `data.ctrl[range]`.
    #[must_use]
    pub fn ctrl(mut self, range: Range<usize>) -> Self {
        self.entries
            .push(BuilderEntry::Resolved(Extractor::Ctrl(range)));
        self
    }

    /// Observe `data.sensordata[range]`.
    #[must_use]
    pub fn sensordata(mut self, range: Range<usize>) -> Self {
        self.entries
            .push(BuilderEntry::Resolved(Extractor::Sensordata(range)));
        self
    }

    /// Observe `data.actuator_force[range]`.
    #[must_use]
    pub fn actuator_force(mut self, range: Range<usize>) -> Self {
        self.entries
            .push(BuilderEntry::Resolved(Extractor::ActuatorForce(range)));
        self
    }

    /// Observe `data.qfrc_constraint[range]`.
    #[must_use]
    pub fn qfrc_constraint(mut self, range: Range<usize>) -> Self {
        self.entries
            .push(BuilderEntry::Resolved(Extractor::QfrcConstraint(range)));
        self
    }

    // ── per-body fields (body-index ranges) ──────────────────────────────

    /// Observe body positions: 3 floats per body in `body_range`.
    #[must_use]
    pub fn xpos(mut self, body_range: Range<usize>) -> Self {
        self.entries
            .push(BuilderEntry::Resolved(Extractor::Xpos(body_range)));
        self
    }

    /// Observe site positions: 3 floats per site in `site_range`.
    #[must_use]
    pub fn site_xpos(mut self, site_range: Range<usize>) -> Self {
        self.entries
            .push(BuilderEntry::Resolved(Extractor::SiteXpos(site_range)));
        self
    }

    /// Observe body orientations: 4 floats (w, i, j, k) per body in `body_range`.
    #[must_use]
    pub fn xquat(mut self, body_range: Range<usize>) -> Self {
        self.entries
            .push(BuilderEntry::Resolved(Extractor::Xquat(body_range)));
        self
    }

    /// Observe body spatial velocities: 6 floats per body in `body_range`.
    #[must_use]
    pub fn cvel(mut self, body_range: Range<usize>) -> Self {
        self.entries
            .push(BuilderEntry::Resolved(Extractor::Cvel(body_range)));
        self
    }

    // ── by name ──────────────────────────────────────────────────────────

    /// Observe a sensor by name.  Resolved at `build()` time via
    /// `model.sensor_name_to_id`.
    #[must_use]
    pub fn sensor(mut self, name: &str) -> Self {
        self.entries
            .push(BuilderEntry::SensorByName(name.to_owned()));
        self
    }

    // ── scalars ──────────────────────────────────────────────────────────

    /// Observe `data.ncon` as a single `f32`.
    #[must_use]
    pub fn contact_count(mut self) -> Self {
        self.entries
            .push(BuilderEntry::Resolved(Extractor::ContactCount));
        self
    }

    /// Observe `data.time` as a single `f32`.
    #[must_use]
    pub fn time(mut self) -> Self {
        self.entries.push(BuilderEntry::Resolved(Extractor::Time));
        self
    }

    /// Observe `[kinetic_energy, potential_energy]` (2 floats).
    #[must_use]
    pub fn energy(mut self) -> Self {
        self.entries.push(BuilderEntry::Resolved(Extractor::Energy));
        self
    }

    // ── shorthands ───────────────────────────────────────────────────────

    /// Observe all joint positions (`0..nq`).  Expanded at `build()` time.
    #[must_use]
    pub fn all_qpos(mut self) -> Self {
        self.entries.push(BuilderEntry::AllQpos);
        self
    }

    /// Observe all joint velocities (`0..nv`).  Expanded at `build()` time.
    #[must_use]
    pub fn all_qvel(mut self) -> Self {
        self.entries.push(BuilderEntry::AllQvel);
        self
    }

    /// Observe all sensor data (`0..nsensordata`).  Expanded at `build()` time.
    #[must_use]
    pub fn all_sensordata(mut self) -> Self {
        self.entries.push(BuilderEntry::AllSensordata);
        self
    }

    // ── build ────────────────────────────────────────────────────────────

    /// Validate all ranges against the model and freeze the space.
    ///
    /// # Errors
    ///
    /// Returns [`SpaceError`] if any range is out of bounds or a sensor
    /// name is not found.
    pub fn build(self, model: &Model) -> Result<ObservationSpace, SpaceError> {
        let mut extractors = Vec::with_capacity(self.entries.len());
        let mut segments = Vec::with_capacity(self.entries.len());
        let mut offset = 0;

        for entry in self.entries {
            let (ext, label) = match entry {
                BuilderEntry::Resolved(ext) => {
                    validate_extractor(&ext, model)?;
                    let label = extractor_label(&ext);
                    (ext, label)
                }
                BuilderEntry::SensorByName(ref name) => {
                    let ext = resolve_sensor(name, model)?;
                    let label = format!("sensor({name})");
                    (ext, label)
                }
                BuilderEntry::AllQpos => {
                    let ext = Extractor::Qpos(0..model.nq);
                    (ext, format!("qpos(0..{})", model.nq))
                }
                BuilderEntry::AllQvel => {
                    let ext = Extractor::Qvel(0..model.nv);
                    (ext, format!("qvel(0..{})", model.nv))
                }
                BuilderEntry::AllSensordata => {
                    let ext = Extractor::Sensordata(0..model.nsensordata);
                    (ext, format!("sensordata(0..{})", model.nsensordata))
                }
            };
            let dim = ext.dim();
            segments.push(ObsSegment { label, offset, dim });
            offset += dim;
            extractors.push(ext);
        }

        let dim = offset;

        Ok(ObservationSpace {
            extractors,
            segments,
            dim,
        })
    }
}

// ─── validation helpers ─────────────────────────────────────────────────────

/// Produce a human-readable label for an extractor.
fn extractor_label(ext: &Extractor) -> String {
    match ext {
        Extractor::Qpos(r) => format!("qpos({}..{})", r.start, r.end),
        Extractor::Qvel(r) => format!("qvel({}..{})", r.start, r.end),
        Extractor::Qacc(r) => format!("qacc({}..{})", r.start, r.end),
        Extractor::Ctrl(r) => format!("ctrl({}..{})", r.start, r.end),
        Extractor::Sensordata(r) => format!("sensordata({}..{})", r.start, r.end),
        Extractor::ActuatorForce(r) => format!("actuator_force({}..{})", r.start, r.end),
        Extractor::QfrcConstraint(r) => format!("qfrc_constraint({}..{})", r.start, r.end),
        Extractor::Xpos(r) => format!("xpos({}..{})", r.start, r.end),
        Extractor::SiteXpos(r) => format!("site_xpos({}..{})", r.start, r.end),
        Extractor::Xquat(r) => format!("xquat({}..{})", r.start, r.end),
        Extractor::Cvel(r) => format!("cvel({}..{})", r.start, r.end),
        Extractor::ContactCount => "contact_count".to_owned(),
        Extractor::Time => "time".to_owned(),
        Extractor::Energy => "energy".to_owned(),
    }
}

/// Validate that an extractor's range fits the model's dimensions.
fn validate_extractor(ext: &Extractor, model: &Model) -> Result<(), SpaceError> {
    match ext {
        Extractor::Qpos(r) => check_flat("qpos", r, model.nq),
        Extractor::Qvel(r) | Extractor::Qacc(r) | Extractor::QfrcConstraint(r) => check_flat(
            match ext {
                Extractor::Qvel(_) => "qvel",
                Extractor::Qacc(_) => "qacc",
                _ => "qfrc_constraint",
            },
            r,
            model.nv,
        ),
        Extractor::Ctrl(r) | Extractor::ActuatorForce(r) => check_flat(
            if matches!(ext, Extractor::Ctrl(_)) {
                "ctrl"
            } else {
                "actuator_force"
            },
            r,
            model.nu,
        ),
        Extractor::Sensordata(r) => check_flat("sensordata", r, model.nsensordata),

        Extractor::Xpos(r) => check_body("xpos", r, model.nbody),
        Extractor::SiteXpos(r) => check_site("site_xpos", r, model.nsite),
        Extractor::Xquat(r) => check_body("xquat", r, model.nbody),
        Extractor::Cvel(r) => check_body("cvel", r, model.nbody),

        // Scalars — always valid.
        Extractor::ContactCount | Extractor::Time | Extractor::Energy => Ok(()),
    }
}

/// Check an element-index range against a flat field length.
fn check_flat(
    field: &'static str,
    range: &Range<usize>,
    field_len: usize,
) -> Result<(), SpaceError> {
    if range.end > field_len {
        return Err(SpaceError::RangeOutOfBounds {
            field,
            range: range.clone(),
            field_len,
        });
    }
    Ok(())
}

/// Check a body-index range against `nbody`.
fn check_body(field: &'static str, range: &Range<usize>, nbody: usize) -> Result<(), SpaceError> {
    if range.end > nbody {
        return Err(SpaceError::BodyRangeOutOfBounds {
            field,
            range: range.clone(),
            nbody,
        });
    }
    Ok(())
}

/// Check a site-index range against `nsite`.
fn check_site(field: &'static str, range: &Range<usize>, nsite: usize) -> Result<(), SpaceError> {
    if range.end > nsite {
        return Err(SpaceError::SiteRangeOutOfBounds {
            field,
            range: range.clone(),
            nsite,
        });
    }
    Ok(())
}

/// Resolve a sensor name to a `Sensordata` extractor.
fn resolve_sensor(name: &str, model: &Model) -> Result<Extractor, SpaceError> {
    let id = model
        .sensor_name_to_id
        .get(name)
        .ok_or_else(|| SpaceError::SensorNotFound {
            name: name.to_owned(),
        })?;
    let adr = model.sensor_adr[*id];
    let dim = model.sensor_dim[*id];
    Ok(Extractor::Sensordata(adr..adr + dim))
}

// ─── Injector ───────────────────────────────────────────────────────────────

/// A single injection: "write these action elements into this field of Data."
///
/// Flat fields use element indices.  Per-body fields use body indices and
/// unflatten automatically (6 per body for `xfrc_applied`, 3 for `mocap_pos`,
/// 4 for `mocap_quat`).
#[derive(Debug, Clone)]
enum Injector {
    /// `action[offset..]` → `data.ctrl[range]`.  Clamped to `actuator_ctrlrange`.
    Ctrl(Range<usize>),
    /// `action[offset..]` → `data.qfrc_applied[range]`.
    QfrcApplied(Range<usize>),
    /// `action[offset..]` → `data.xfrc_applied[body_range]` (6 per body).
    XfrcApplied(Range<usize>),
    /// `action[offset..]` → `data.mocap_pos[body_range]` (3 per body).
    MocapPos(Range<usize>),
    /// `action[offset..]` → `data.mocap_quat[body_range]` (4 per body).
    MocapQuat(Range<usize>),
}

impl Injector {
    /// Number of `f32` elements this injector consumes from the action tensor.
    fn dim(&self) -> usize {
        match self {
            Self::Ctrl(r) | Self::QfrcApplied(r) => r.len(),
            Self::XfrcApplied(r) => r.len() * 6,
            Self::MocapPos(r) => r.len() * 3,
            Self::MocapQuat(r) => r.len() * 4,
        }
    }

    /// Inject values from `action_slice` (f32) into `data`, clamping Ctrl.
    #[allow(clippy::cast_possible_truncation)]
    fn inject(&self, action_slice: &[f32], data: &mut Data, model: &Model) {
        match self {
            Self::Ctrl(r) => {
                for (i, &val) in r.clone().zip(action_slice.iter()) {
                    let v = f64::from(val);
                    let (lo, hi) = model.actuator_ctrlrange[i];
                    data.ctrl[i] = v.clamp(lo, hi);
                }
            }
            Self::QfrcApplied(r) => {
                for (i, &val) in r.clone().zip(action_slice.iter()) {
                    data.qfrc_applied[i] = f64::from(val);
                }
            }
            Self::XfrcApplied(r) => {
                let mut offset = 0;
                for body in r.clone() {
                    for k in 0..6 {
                        data.xfrc_applied[body][k] = f64::from(action_slice[offset + k]);
                    }
                    offset += 6;
                }
            }
            Self::MocapPos(r) => {
                let mut offset = 0;
                for body in r.clone() {
                    data.mocap_pos[body] = nalgebra::Vector3::new(
                        f64::from(action_slice[offset]),
                        f64::from(action_slice[offset + 1]),
                        f64::from(action_slice[offset + 2]),
                    );
                    offset += 3;
                }
            }
            Self::MocapQuat(r) => {
                let mut offset = 0;
                for body in r.clone() {
                    let q = nalgebra::Quaternion::new(
                        f64::from(action_slice[offset]),     // w
                        f64::from(action_slice[offset + 1]), // i
                        f64::from(action_slice[offset + 2]), // j
                        f64::from(action_slice[offset + 3]), // k
                    );
                    // Normalize to unit quaternion (policies output noisy values).
                    data.mocap_quat[body] = nalgebra::UnitQuaternion::new_normalize(q);
                    offset += 4;
                }
            }
        }
    }
}

// ─── ActionSpace ────────────────────────────────────────────────────────────

/// Selects which slices of `Data` the action vector writes to.
///
/// The action tensor is a flat `f32` vector.  Each injector consumes a
/// contiguous slice of that vector and writes it to the corresponding
/// `Data` field, casting `f32→f64`.  `Ctrl` injectors additionally clamp
/// to `model.actuator_ctrlrange`.
///
/// Constructed via [`ActionSpace::builder()`].
#[derive(Debug, Clone)]
#[cfg_attr(feature = "bevy", derive(bevy_ecs::prelude::Resource))]
pub struct ActionSpace {
    injectors: Vec<(usize, Injector)>, // (offset_in_action, injector)
    dim: usize,
}

impl ActionSpace {
    /// Start building an action space.
    #[must_use]
    pub const fn builder() -> ActionSpaceBuilder {
        ActionSpaceBuilder {
            entries: Vec::new(),
        }
    }

    /// Total action dimension (number of `f32` elements).
    #[must_use]
    pub const fn dim(&self) -> usize {
        self.dim
    }

    /// A [`TensorSpec`] describing this action space.
    ///
    /// For `Ctrl` injectors the bounds come from `model.actuator_ctrlrange`.
    /// Other injectors are unbounded (`-inf..inf`).
    #[must_use]
    #[allow(clippy::cast_possible_truncation)]
    pub fn spec(&self, model: &Model) -> TensorSpec {
        let mut low = vec![f32::NEG_INFINITY; self.dim];
        let mut high = vec![f32::INFINITY; self.dim];
        for (offset, inj) in &self.injectors {
            if let Injector::Ctrl(r) = inj {
                for (k, i) in r.clone().enumerate() {
                    let (lo, hi) = model.actuator_ctrlrange[i];
                    low[offset + k] = lo as f32;
                    high[offset + k] = hi as f32;
                }
            }
        }
        TensorSpec {
            shape: vec![self.dim],
            low: Some(low),
            high: Some(high),
        }
    }

    /// Apply an action tensor to a single `Data` instance.
    ///
    /// Casts `f32→f64`.  Clamps `Ctrl` injectors to
    /// `model.actuator_ctrlrange`.
    pub fn apply(&self, action: &Tensor, data: &mut Data, model: &Model) {
        let slice = action.as_slice();
        for (offset, inj) in &self.injectors {
            let n = inj.dim();
            inj.inject(&slice[*offset..*offset + n], data, model);
        }
    }

    /// Apply batched actions `[n, dim]` to `N` `Data` instances.
    pub fn apply_batch<'a>(
        &self,
        actions: &Tensor,
        envs: impl ExactSizeIterator<Item = &'a mut Data>,
        model: &Model,
    ) {
        for (i, data) in envs.enumerate() {
            let row = actions.row(i);
            for (offset, inj) in &self.injectors {
                let n = inj.dim();
                inj.inject(&row[*offset..*offset + n], data, model);
            }
        }
    }
}

// ─── ActionSpaceBuilder ─────────────────────────────────────────────────────

/// A pending entry in the action builder.
#[derive(Debug, Clone)]
enum ActionBuilderEntry {
    /// Already resolved to a concrete injector.
    Resolved(Injector),
    /// Shorthand: expand to `Ctrl(0..nu)` at `build()` time.
    AllCtrl,
}

/// Builder for [`ActionSpace`].
///
/// Add injectors in the order you want them laid out in the action vector.
/// Call [`build()`](Self::build) to validate all ranges against the model.
#[derive(Debug, Clone)]
pub struct ActionSpaceBuilder {
    entries: Vec<ActionBuilderEntry>,
}

impl ActionSpaceBuilder {
    /// Inject `action[..]` → `data.ctrl[range]`.  Clamped at apply time.
    #[must_use]
    pub fn ctrl(mut self, range: Range<usize>) -> Self {
        self.entries
            .push(ActionBuilderEntry::Resolved(Injector::Ctrl(range)));
        self
    }

    /// Inject into all ctrl channels (`0..nu`).  Expanded at `build()` time.
    #[must_use]
    pub fn all_ctrl(mut self) -> Self {
        self.entries.push(ActionBuilderEntry::AllCtrl);
        self
    }

    /// Inject `action[..]` → `data.qfrc_applied[range]`.
    #[must_use]
    pub fn qfrc_applied(mut self, range: Range<usize>) -> Self {
        self.entries
            .push(ActionBuilderEntry::Resolved(Injector::QfrcApplied(range)));
        self
    }

    /// Inject `action[..]` → `data.xfrc_applied[body_range]` (6 per body).
    #[must_use]
    pub fn xfrc_applied(mut self, body_range: Range<usize>) -> Self {
        self.entries
            .push(ActionBuilderEntry::Resolved(Injector::XfrcApplied(
                body_range,
            )));
        self
    }

    /// Inject `action[..]` → `data.mocap_pos[body_range]` (3 per body).
    #[must_use]
    pub fn mocap_pos(mut self, body_range: Range<usize>) -> Self {
        self.entries
            .push(ActionBuilderEntry::Resolved(Injector::MocapPos(body_range)));
        self
    }

    /// Inject `action[..]` → `data.mocap_quat[body_range]` (4 per body).
    #[must_use]
    pub fn mocap_quat(mut self, body_range: Range<usize>) -> Self {
        self.entries
            .push(ActionBuilderEntry::Resolved(Injector::MocapQuat(
                body_range,
            )));
        self
    }

    /// Validate all ranges against the model and freeze the space.
    ///
    /// # Errors
    ///
    /// Returns [`SpaceError`] if any range is out of bounds.
    pub fn build(self, model: &Model) -> Result<ActionSpace, SpaceError> {
        let mut injectors = Vec::with_capacity(self.entries.len());
        let mut offset = 0;

        for entry in self.entries {
            let inj = match entry {
                ActionBuilderEntry::Resolved(inj) => {
                    validate_injector(&inj, model)?;
                    inj
                }
                ActionBuilderEntry::AllCtrl => Injector::Ctrl(0..model.nu),
            };
            let d = inj.dim();
            injectors.push((offset, inj));
            offset += d;
        }

        Ok(ActionSpace {
            injectors,
            dim: offset,
        })
    }
}

/// Validate that an injector's range fits the model's dimensions.
fn validate_injector(inj: &Injector, model: &Model) -> Result<(), SpaceError> {
    match inj {
        Injector::Ctrl(r) => check_flat("ctrl", r, model.nu),
        Injector::QfrcApplied(r) => check_flat("qfrc_applied", r, model.nv),
        Injector::XfrcApplied(r) => check_body("xfrc_applied", r, model.nbody),
        Injector::MocapPos(r) => check_mocap("mocap_pos", r, model.nmocap),
        Injector::MocapQuat(r) => check_mocap("mocap_quat", r, model.nmocap),
    }
}

/// Check a mocap-body range against `nmocap`.
fn check_mocap(field: &'static str, range: &Range<usize>, nmocap: usize) -> Result<(), SpaceError> {
    if range.end > nmocap {
        return Err(SpaceError::MocapRangeOutOfBounds {
            field,
            range: range.clone(),
            nmocap,
        });
    }
    Ok(())
}

// ─── tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use
)]
mod tests {
    use super::*;

    /// Build a minimal pendulum model+data for testing extractors.
    ///
    /// One hinge joint (nq=1, nv=1), one actuator, one sensor ("angle",
    /// dim=1 at sensordata[0]), two bodies (world + pendulum).
    fn pendulum() -> (Model, Data) {
        let xml = r#"
        <mujoco>
          <worldbody>
            <body name="pendulum" pos="0 0 1">
              <joint name="hinge" type="hinge" axis="0 1 0"/>
              <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5"/>
              <site name="tip" pos="0 0 -0.5"/>
            </body>
          </worldbody>
          <actuator>
            <motor joint="hinge" name="motor"/>
          </actuator>
          <sensor>
            <jointpos joint="hinge" name="angle"/>
          </sensor>
        </mujoco>
        "#;
        let model = sim_mjcf::load_model(xml).expect("valid MJCF");
        let data = model.make_data();
        (model, data)
    }

    // ── flat field extractors ────────────────────────────────────────────

    #[test]
    fn extract_qpos() {
        let (model, mut data) = pendulum();
        data.qpos[0] = 1.5;
        let space = ObservationSpace::builder()
            .qpos(0..1)
            .build(&model)
            .unwrap();
        let obs = space.extract(&data);
        assert_eq!(obs.shape(), &[1]);
        assert_eq!(obs.as_slice(), &[1.5_f64 as f32]);
    }

    #[test]
    fn extract_qvel() {
        let (model, mut data) = pendulum();
        data.qvel[0] = -3.0;
        let space = ObservationSpace::builder()
            .qvel(0..1)
            .build(&model)
            .unwrap();
        let obs = space.extract(&data);
        assert_eq!(obs.as_slice(), &[-3.0_f64 as f32]);
    }

    #[test]
    fn extract_qacc() {
        let (model, mut data) = pendulum();
        data.qacc[0] = 0.25;
        let space = ObservationSpace::builder()
            .qacc(0..1)
            .build(&model)
            .unwrap();
        let obs = space.extract(&data);
        assert_eq!(obs.as_slice(), &[0.25_f64 as f32]);
    }

    #[test]
    fn extract_ctrl() {
        let (model, mut data) = pendulum();
        data.ctrl[0] = 5.0;
        let space = ObservationSpace::builder()
            .ctrl(0..1)
            .build(&model)
            .unwrap();
        let obs = space.extract(&data);
        assert_eq!(obs.as_slice(), &[5.0_f32]);
    }

    #[test]
    fn extract_sensordata() {
        let (model, mut data) = pendulum();
        data.sensordata[0] = 0.75;
        let space = ObservationSpace::builder()
            .sensordata(0..1)
            .build(&model)
            .unwrap();
        let obs = space.extract(&data);
        assert_eq!(obs.as_slice(), &[0.75_f64 as f32]);
    }

    #[test]
    fn extract_actuator_force() {
        let (model, mut data) = pendulum();
        data.actuator_force[0] = 2.5;
        let space = ObservationSpace::builder()
            .actuator_force(0..1)
            .build(&model)
            .unwrap();
        let obs = space.extract(&data);
        assert_eq!(obs.as_slice(), &[2.5_f64 as f32]);
    }

    #[test]
    fn extract_qfrc_constraint() {
        let (model, mut data) = pendulum();
        data.qfrc_constraint[0] = -1.0;
        let space = ObservationSpace::builder()
            .qfrc_constraint(0..1)
            .build(&model)
            .unwrap();
        let obs = space.extract(&data);
        assert_eq!(obs.as_slice(), &[-1.0_f32]);
    }

    // ── per-body extractors ──────────────────────────────────────────────

    #[test]
    fn extract_xpos() {
        let (model, data) = pendulum();
        // Body 0 = world (0,0,0), Body 1 = pendulum (0,0,1)
        let space = ObservationSpace::builder()
            .xpos(0..2)
            .build(&model)
            .unwrap();
        let obs = space.extract(&data);
        assert_eq!(obs.shape(), &[6]);
        // World body at origin
        assert_eq!(obs.as_slice()[0], 0.0);
        assert_eq!(obs.as_slice()[1], 0.0);
        assert_eq!(obs.as_slice()[2], 0.0);
        // Pendulum body — may differ from pos="0 0 1" because forward()
        // hasn't run, but the extraction itself is what we're testing.
    }

    #[test]
    fn extract_xquat() {
        let (model, data) = pendulum();
        // Extract just body 1 (pendulum)
        let space = ObservationSpace::builder()
            .xquat(1..2)
            .build(&model)
            .unwrap();
        let obs = space.extract(&data);
        assert_eq!(obs.shape(), &[4]); // w, i, j, k
    }

    #[test]
    fn extract_cvel() {
        let (model, data) = pendulum();
        let space = ObservationSpace::builder()
            .cvel(0..2)
            .build(&model)
            .unwrap();
        let obs = space.extract(&data);
        assert_eq!(obs.shape(), &[12]); // 2 bodies × 6
        // At rest, all zeros
        assert!(obs.as_slice().iter().all(|&v| v == 0.0));
    }

    // ── per-site extractors ─────────────────────────────────────────────

    #[test]
    fn extract_site_xpos() {
        let (model, data) = pendulum();
        // Pendulum has 1 site: "tip" at pos="0 0 -0.5" on body 1.
        assert_eq!(model.nsite, 1);
        let space = ObservationSpace::builder()
            .site_xpos(0..1)
            .build(&model)
            .unwrap();
        let obs = space.extract(&data);
        assert_eq!(obs.shape(), &[3]); // 1 site × 3 floats
        // Position depends on FK state — just verify shape and finite values.
        assert!(obs.as_slice().iter().all(|v| v.is_finite()));
    }

    // ── scalar extractors ────────────────────────────────────────────────

    #[test]
    fn extract_contact_count() {
        let (model, data) = pendulum();
        let space = ObservationSpace::builder()
            .contact_count()
            .build(&model)
            .unwrap();
        let obs = space.extract(&data);
        assert_eq!(obs.shape(), &[1]);
        assert_eq!(obs.as_slice()[0], 0.0); // no contacts initially
    }

    #[test]
    fn extract_time() {
        let (model, mut data) = pendulum();
        data.time = 1.234;
        let space = ObservationSpace::builder().time().build(&model).unwrap();
        let obs = space.extract(&data);
        assert_eq!(obs.as_slice(), &[1.234_f64 as f32]);
    }

    #[test]
    fn extract_energy() {
        let (model, mut data) = pendulum();
        data.energy_kinetic = 1.5;
        data.energy_potential = -9.81;
        let space = ObservationSpace::builder().energy().build(&model).unwrap();
        let obs = space.extract(&data);
        assert_eq!(obs.shape(), &[2]);
        assert_eq!(obs.as_slice()[0], 1.5_f64 as f32);
        assert_eq!(obs.as_slice()[1], -9.81_f64 as f32);
    }

    // ── sensor by name ───────────────────────────────────────────────────

    #[test]
    fn extract_sensor_by_name() {
        let (model, mut data) = pendulum();
        data.sensordata[0] = 0.42;
        let space = ObservationSpace::builder()
            .sensor("angle")
            .build(&model)
            .unwrap();
        let obs = space.extract(&data);
        assert_eq!(obs.shape(), &[1]);
        assert_eq!(obs.as_slice()[0], 0.42_f64 as f32);
    }

    #[test]
    fn sensor_not_found_returns_error() {
        let (model, _data) = pendulum();
        let err = ObservationSpace::builder()
            .sensor("nonexistent")
            .build(&model)
            .unwrap_err();
        assert!(matches!(err, SpaceError::SensorNotFound { .. }));
    }

    // ── shorthands ───────────────────────────────────────────────────────

    #[test]
    fn all_qpos_expands_to_full_range() {
        let (model, _data) = pendulum();
        let space = ObservationSpace::builder()
            .all_qpos()
            .build(&model)
            .unwrap();
        assert_eq!(space.dim(), model.nq);
    }

    #[test]
    fn all_qvel_expands_to_full_range() {
        let (model, _data) = pendulum();
        let space = ObservationSpace::builder()
            .all_qvel()
            .build(&model)
            .unwrap();
        assert_eq!(space.dim(), model.nv);
    }

    #[test]
    fn all_sensordata_expands_to_full_range() {
        let (model, _data) = pendulum();
        let space = ObservationSpace::builder()
            .all_sensordata()
            .build(&model)
            .unwrap();
        assert_eq!(space.dim(), model.nsensordata);
    }

    // ── concatenation + dim ──────────────────────────────────────────────

    #[test]
    fn multiple_extractors_concatenate_in_order() {
        let (model, mut data) = pendulum();
        data.qpos[0] = 1.0;
        data.qvel[0] = 2.0;
        data.time = 3.0;
        let space = ObservationSpace::builder()
            .qpos(0..1)
            .qvel(0..1)
            .time()
            .build(&model)
            .unwrap();
        assert_eq!(space.dim(), 3);
        let obs = space.extract(&data);
        assert_eq!(obs.as_slice(), &[1.0_f32, 2.0, 3.0]);
    }

    // ── segments ──────────────────────────────────────────────────────────

    #[test]
    fn segments_labels_offsets_dims() {
        // Build a cart-pole model with enough variety for 5 extractors.
        let xml = r#"
        <mujoco>
          <option>
            <flag energy="enable"/>
          </option>
          <worldbody>
            <body name="cart" pos="0 0 0.5">
              <joint name="slide" type="slide" axis="1 0 0"/>
              <geom size="0.1"/>
              <body name="pole" pos="0 0 0.1">
                <joint name="hinge" type="hinge" axis="0 1 0"/>
                <geom type="capsule" size="0.02" fromto="0 0 0 0 0 0.5"/>
              </body>
            </body>
          </worldbody>
          <actuator>
            <motor joint="slide" name="force"/>
          </actuator>
          <sensor>
            <jointpos joint="slide" name="cart_pos"/>
            <jointvel joint="hinge" name="pole_vel"/>
          </sensor>
        </mujoco>
        "#;
        let model = sim_mjcf::load_model(xml).expect("valid MJCF");

        let space = ObservationSpace::builder()
            .all_qpos()                     // 1: qpos(0..2)
            .all_qvel()                     // 2: qvel(0..2)
            .sensor("cart_pos")             // 3: sensor(cart_pos)
            .sensor("pole_vel")             // 4: sensor(pole_vel)
            .energy()                       // 5: energy
            .build(&model)
            .unwrap();

        let segs = space.segments();
        assert_eq!(segs.len(), 5);

        // Segment 0: all_qpos → qpos(0..2), offset=0, dim=2
        assert_eq!(segs[0].label, "qpos(0..2)");
        assert_eq!(segs[0].offset, 0);
        assert_eq!(segs[0].dim, 2);

        // Segment 1: all_qvel → qvel(0..2), offset=2, dim=2
        assert_eq!(segs[1].label, "qvel(0..2)");
        assert_eq!(segs[1].offset, 2);
        assert_eq!(segs[1].dim, 2);

        // Segment 2: sensor("cart_pos"), offset=4, dim=1
        assert_eq!(segs[2].label, "sensor(cart_pos)");
        assert_eq!(segs[2].offset, 4);
        assert_eq!(segs[2].dim, 1);

        // Segment 3: sensor("pole_vel"), offset=5, dim=1
        assert_eq!(segs[3].label, "sensor(pole_vel)");
        assert_eq!(segs[3].offset, 5);
        assert_eq!(segs[3].dim, 1);

        // Segment 4: energy, offset=6, dim=2
        assert_eq!(segs[4].label, "energy");
        assert_eq!(segs[4].offset, 6);
        assert_eq!(segs[4].dim, 2);

        // Total dim should match sum of segment dims.
        let sum: usize = segs.iter().map(|s| s.dim).sum();
        assert_eq!(space.dim(), sum);
        assert_eq!(space.dim(), 8);
    }

    // ── spec ─────────────────────────────────────────────────────────────

    #[test]
    fn spec_matches_dim() {
        let (model, _data) = pendulum();
        let space = ObservationSpace::builder()
            .all_qpos()
            .all_qvel()
            .contact_count()
            .build(&model)
            .unwrap();
        let spec = space.spec();
        assert_eq!(spec.shape, vec![space.dim()]);
        assert!(spec.low.is_none());
        assert!(spec.high.is_none());
    }

    // ── batch extraction ─────────────────────────────────────────────────

    #[test]
    fn extract_batch_shape_and_values() {
        let (model, mut d1) = pendulum();
        d1.qpos[0] = 1.0;
        let mut d2 = model.make_data();
        d2.qpos[0] = 2.0;

        let space = ObservationSpace::builder()
            .qpos(0..1)
            .build(&model)
            .unwrap();

        let batch = space.extract_batch([&d1, &d2].into_iter());
        assert_eq!(batch.shape(), &[2, 1]);
        assert_eq!(batch.row(0), &[1.0_f32]);
        assert_eq!(batch.row(1), &[2.0_f32]);
    }

    #[test]
    fn extract_batch_matches_individual_extracts() {
        let (model, mut d1) = pendulum();
        d1.qpos[0] = 0.5;
        d1.qvel[0] = -1.0;
        let mut d2 = model.make_data();
        d2.qpos[0] = -0.3;
        d2.qvel[0] = 2.0;

        let space = ObservationSpace::builder()
            .all_qpos()
            .all_qvel()
            .time()
            .build(&model)
            .unwrap();

        let obs1 = space.extract(&d1);
        let obs2 = space.extract(&d2);
        let batch = space.extract_batch([&d1, &d2].into_iter());

        assert_eq!(batch.row(0), obs1.as_slice());
        assert_eq!(batch.row(1), obs2.as_slice());
    }

    // ── validation errors ────────────────────────────────────────────────

    #[test]
    fn qpos_range_out_of_bounds() {
        let (model, _data) = pendulum();
        let err = ObservationSpace::builder()
            .qpos(0..100)
            .build(&model)
            .unwrap_err();
        assert!(matches!(
            err,
            SpaceError::RangeOutOfBounds { field: "qpos", .. }
        ));
    }

    #[test]
    fn qvel_range_out_of_bounds() {
        let (model, _data) = pendulum();
        let err = ObservationSpace::builder()
            .qvel(0..100)
            .build(&model)
            .unwrap_err();
        assert!(matches!(
            err,
            SpaceError::RangeOutOfBounds { field: "qvel", .. }
        ));
    }

    #[test]
    fn xpos_body_range_out_of_bounds() {
        let (model, _data) = pendulum();
        let err = ObservationSpace::builder()
            .xpos(0..100)
            .build(&model)
            .unwrap_err();
        assert!(matches!(
            err,
            SpaceError::BodyRangeOutOfBounds { field: "xpos", .. }
        ));
    }

    #[test]
    fn xquat_body_range_out_of_bounds() {
        let (model, _data) = pendulum();
        let err = ObservationSpace::builder()
            .xquat(0..100)
            .build(&model)
            .unwrap_err();
        assert!(matches!(
            err,
            SpaceError::BodyRangeOutOfBounds { field: "xquat", .. }
        ));
    }

    #[test]
    fn cvel_body_range_out_of_bounds() {
        let (model, _data) = pendulum();
        let err = ObservationSpace::builder()
            .cvel(0..100)
            .build(&model)
            .unwrap_err();
        assert!(matches!(
            err,
            SpaceError::BodyRangeOutOfBounds { field: "cvel", .. }
        ));
    }

    #[test]
    fn site_xpos_range_out_of_bounds() {
        let (model, _data) = pendulum();
        let err = ObservationSpace::builder()
            .site_xpos(0..100)
            .build(&model)
            .unwrap_err();
        assert!(matches!(
            err,
            SpaceError::SiteRangeOutOfBounds {
                field: "site_xpos",
                ..
            }
        ));
    }

    #[test]
    fn ctrl_range_out_of_bounds() {
        let (model, _data) = pendulum();
        let err = ObservationSpace::builder()
            .ctrl(0..100)
            .build(&model)
            .unwrap_err();
        assert!(matches!(
            err,
            SpaceError::RangeOutOfBounds { field: "ctrl", .. }
        ));
    }

    #[test]
    fn sensordata_range_out_of_bounds() {
        let (model, _data) = pendulum();
        let err = ObservationSpace::builder()
            .sensordata(0..100)
            .build(&model)
            .unwrap_err();
        assert!(matches!(
            err,
            SpaceError::RangeOutOfBounds {
                field: "sensordata",
                ..
            }
        ));
    }

    #[test]
    fn actuator_force_range_out_of_bounds() {
        let (model, _data) = pendulum();
        let err = ObservationSpace::builder()
            .actuator_force(0..100)
            .build(&model)
            .unwrap_err();
        assert!(matches!(
            err,
            SpaceError::RangeOutOfBounds {
                field: "actuator_force",
                ..
            }
        ));
    }

    // ── f64→f32 cast correctness ─────────────────────────────────────────

    #[test]
    fn f64_to_f32_cast_matches_direct_access() {
        let (model, mut data) = pendulum();
        let val: f64 = std::f64::consts::PI;
        data.qpos[0] = val;
        let space = ObservationSpace::builder()
            .qpos(0..1)
            .build(&model)
            .unwrap();
        let obs = space.extract(&data);
        assert_eq!(obs.as_slice()[0], val as f32);
    }

    // ══════════════════════════════════════════════════════════════════════
    //  ActionSpace tests
    // ══════════════════════════════════════════════════════════════════════

    /// Build a pendulum with ctrllimited motor for clamping tests.
    fn pendulum_clamped() -> (Model, Data) {
        let xml = r#"
        <mujoco>
          <worldbody>
            <body name="pendulum" pos="0 0 1">
              <joint name="hinge" type="hinge" axis="0 1 0"/>
              <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5"/>
            </body>
          </worldbody>
          <actuator>
            <motor joint="hinge" name="motor" ctrllimited="true" ctrlrange="-1 1"/>
          </actuator>
        </mujoco>
        "#;
        let model = sim_mjcf::load_model(xml).expect("valid MJCF");
        let data = model.make_data();
        (model, data)
    }

    // ── ctrl injection + round-trip ──────────────────────────────────────

    #[test]
    fn apply_ctrl_roundtrip() {
        let (model, mut data) = pendulum();
        let space = ActionSpace::builder().ctrl(0..1).build(&model).unwrap();
        let action = Tensor::from_slice(&[0.75_f32], &[1]);
        space.apply(&action, &mut data, &model);
        assert_eq!(data.ctrl[0], f64::from(0.75_f32));
    }

    #[test]
    fn apply_ctrl_clamping() {
        let (model, mut data) = pendulum_clamped();
        let space = ActionSpace::builder().all_ctrl().build(&model).unwrap();
        // Action exceeds ctrlrange [-1, 1]
        let action = Tensor::from_slice(&[5.0_f32], &[1]);
        space.apply(&action, &mut data, &model);
        assert_eq!(data.ctrl[0], 1.0);

        let action = Tensor::from_slice(&[-5.0_f32], &[1]);
        space.apply(&action, &mut data, &model);
        assert_eq!(data.ctrl[0], -1.0);
    }

    #[test]
    fn apply_ctrl_within_range_not_clamped() {
        let (model, mut data) = pendulum_clamped();
        let space = ActionSpace::builder().all_ctrl().build(&model).unwrap();
        let action = Tensor::from_slice(&[0.5_f32], &[1]);
        space.apply(&action, &mut data, &model);
        assert_eq!(data.ctrl[0], f64::from(0.5_f32));
    }

    // ── qfrc_applied injection ───────────────────────────────────────────

    #[test]
    fn apply_qfrc_applied_roundtrip() {
        let (model, mut data) = pendulum();
        let space = ActionSpace::builder()
            .qfrc_applied(0..1)
            .build(&model)
            .unwrap();
        let action = Tensor::from_slice(&[2.71_f32], &[1]);
        space.apply(&action, &mut data, &model);
        assert_eq!(data.qfrc_applied[0], f64::from(2.71_f32));
    }

    // ── xfrc_applied injection ───────────────────────────────────────────

    #[test]
    fn apply_xfrc_applied_roundtrip() {
        let (model, mut data) = pendulum();
        // Body 1 = pendulum
        let space = ActionSpace::builder()
            .xfrc_applied(1..2)
            .build(&model)
            .unwrap();
        assert_eq!(space.dim(), 6);
        let vals = [1.0_f32, 2.0, 3.0, 4.0, 5.0, 6.0];
        let action = Tensor::from_slice(&vals, &[6]);
        space.apply(&action, &mut data, &model);
        for (k, &val) in vals.iter().enumerate() {
            assert_eq!(data.xfrc_applied[1][k], f64::from(val));
        }
    }

    // ── mocap_pos injection ─────────────────────────────────────────────��

    fn pendulum_mocap() -> (Model, Data) {
        let xml = r#"
        <mujoco>
          <worldbody>
            <body name="target" mocap="true" pos="0.5 0 0.5">
              <geom type="sphere" size="0.05" contype="0" conaffinity="0"/>
            </body>
            <body name="pendulum" pos="0 0 1">
              <joint name="hinge" type="hinge" axis="0 1 0"/>
              <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5"/>
            </body>
          </worldbody>
          <actuator>
            <motor joint="hinge" name="motor"/>
          </actuator>
        </mujoco>
        "#;
        let model = sim_mjcf::load_model(xml).expect("valid MJCF");
        let data = model.make_data();
        (model, data)
    }

    #[test]
    fn apply_mocap_pos_roundtrip() {
        let (model, mut data) = pendulum_mocap();
        assert_eq!(model.nmocap, 1);
        let space = ActionSpace::builder()
            .mocap_pos(0..1)
            .build(&model)
            .unwrap();
        assert_eq!(space.dim(), 3);
        let action = Tensor::from_slice(&[1.0_f32, 2.0, 3.0], &[3]);
        space.apply(&action, &mut data, &model);
        assert_eq!(data.mocap_pos[0][0], f64::from(1.0_f32));
        assert_eq!(data.mocap_pos[0][1], f64::from(2.0_f32));
        assert_eq!(data.mocap_pos[0][2], f64::from(3.0_f32));
    }

    // ── mocap_quat injection ─────────────────────────────────────────────

    #[test]
    fn apply_mocap_quat_roundtrip() {
        let (model, mut data) = pendulum_mocap();
        let space = ActionSpace::builder()
            .mocap_quat(0..1)
            .build(&model)
            .unwrap();
        assert_eq!(space.dim(), 4);
        // Approximate identity quaternion (will be normalized)
        let action = Tensor::from_slice(&[1.0_f32, 0.0, 0.0, 0.0], &[4]);
        space.apply(&action, &mut data, &model);
        let q = &data.mocap_quat[0];
        assert!((q.w - 1.0).abs() < 1e-6);
        assert!(q.i.abs() < 1e-6);
        assert!(q.j.abs() < 1e-6);
        assert!(q.k.abs() < 1e-6);
    }

    #[test]
    fn apply_mocap_quat_normalizes() {
        let (model, mut data) = pendulum_mocap();
        let space = ActionSpace::builder()
            .mocap_quat(0..1)
            .build(&model)
            .unwrap();
        // Non-unit quaternion — should be normalized
        let action = Tensor::from_slice(&[2.0_f32, 0.0, 0.0, 0.0], &[4]);
        space.apply(&action, &mut data, &model);
        let q = &data.mocap_quat[0];
        let norm = q.quaternion().norm();
        assert!((norm - 1.0).abs() < 1e-10);
    }

    // ── all_ctrl shorthand ───────────────────────────────────────────────

    #[test]
    fn all_ctrl_expands_to_full_range() {
        let (model, _data) = pendulum();
        let space = ActionSpace::builder().all_ctrl().build(&model).unwrap();
        assert_eq!(space.dim(), model.nu);
    }

    // ── dim + spec ───────────────────────────────────────────────────────

    #[test]
    fn action_dim_sums_injectors() {
        let (model, _data) = pendulum();
        // ctrl(1) + qfrc_applied(1) + xfrc_applied(1 body × 6) = 8
        let space = ActionSpace::builder()
            .ctrl(0..1)
            .qfrc_applied(0..1)
            .xfrc_applied(1..2)
            .build(&model)
            .unwrap();
        assert_eq!(space.dim(), 8);
    }

    #[test]
    fn action_spec_has_ctrl_bounds() {
        let (model, _data) = pendulum_clamped();
        let space = ActionSpace::builder().all_ctrl().build(&model).unwrap();
        let spec = space.spec(&model);
        assert_eq!(spec.shape, vec![1]);
        let low = spec.low.unwrap();
        let high = spec.high.unwrap();
        assert_eq!(low[0], -1.0_f32);
        assert_eq!(high[0], 1.0_f32);
    }

    #[test]
    fn action_spec_non_ctrl_unbounded() {
        let (model, _data) = pendulum();
        let space = ActionSpace::builder()
            .qfrc_applied(0..1)
            .build(&model)
            .unwrap();
        let spec = space.spec(&model);
        let low = spec.low.unwrap();
        let high = spec.high.unwrap();
        assert_eq!(low[0], f32::NEG_INFINITY);
        assert_eq!(high[0], f32::INFINITY);
    }

    // ── batch apply ──────────────────────────────────────────────────────

    #[test]
    fn apply_batch_matches_individual_applies() {
        let (model, mut d1) = pendulum();
        let mut d2 = model.make_data();

        let space = ActionSpace::builder().all_ctrl().build(&model).unwrap();

        // Batch apply
        let actions = Tensor::from_slice(&[0.5_f32, -0.3], &[2, 1]);
        space.apply_batch(&actions, [&mut d1, &mut d2].into_iter(), &model);

        assert_eq!(d1.ctrl[0], f64::from(0.5_f32));
        assert_eq!(d2.ctrl[0], f64::from(-0.3_f32));
    }

    #[test]
    fn apply_batch_clamps_ctrl() {
        let (model, mut d1) = pendulum_clamped();
        let mut d2 = model.make_data();

        let space = ActionSpace::builder().all_ctrl().build(&model).unwrap();

        let actions = Tensor::from_slice(&[5.0_f32, -5.0], &[2, 1]);
        space.apply_batch(&actions, [&mut d1, &mut d2].into_iter(), &model);

        assert_eq!(d1.ctrl[0], 1.0);
        assert_eq!(d2.ctrl[0], -1.0);
    }

    // ── validation errors ────────────────────────────────────────────────

    #[test]
    fn action_ctrl_range_out_of_bounds() {
        let (model, _data) = pendulum();
        let err = ActionSpace::builder()
            .ctrl(0..100)
            .build(&model)
            .unwrap_err();
        assert!(matches!(
            err,
            SpaceError::RangeOutOfBounds { field: "ctrl", .. }
        ));
    }

    #[test]
    fn action_qfrc_applied_range_out_of_bounds() {
        let (model, _data) = pendulum();
        let err = ActionSpace::builder()
            .qfrc_applied(0..100)
            .build(&model)
            .unwrap_err();
        assert!(matches!(
            err,
            SpaceError::RangeOutOfBounds {
                field: "qfrc_applied",
                ..
            }
        ));
    }

    #[test]
    fn action_xfrc_applied_range_out_of_bounds() {
        let (model, _data) = pendulum();
        let err = ActionSpace::builder()
            .xfrc_applied(0..100)
            .build(&model)
            .unwrap_err();
        assert!(matches!(
            err,
            SpaceError::BodyRangeOutOfBounds {
                field: "xfrc_applied",
                ..
            }
        ));
    }

    #[test]
    fn action_mocap_pos_range_out_of_bounds() {
        let (model, _data) = pendulum(); // no mocap bodies
        let err = ActionSpace::builder()
            .mocap_pos(0..1)
            .build(&model)
            .unwrap_err();
        assert!(matches!(
            err,
            SpaceError::MocapRangeOutOfBounds {
                field: "mocap_pos",
                ..
            }
        ));
    }

    #[test]
    fn action_mocap_quat_range_out_of_bounds() {
        let (model, _data) = pendulum(); // no mocap bodies
        let err = ActionSpace::builder()
            .mocap_quat(0..1)
            .build(&model)
            .unwrap_err();
        assert!(matches!(
            err,
            SpaceError::MocapRangeOutOfBounds {
                field: "mocap_quat",
                ..
            }
        ));
    }

    // ── f32→f64 cast correctness ─────────────────────────────────────────

    #[test]
    fn f32_to_f64_cast_uses_from() {
        // Verify that f32→f64 via f64::from() is exact (no precision loss)
        let val = 0.1_f32;
        let (model, mut data) = pendulum();
        let space = ActionSpace::builder()
            .qfrc_applied(0..1)
            .build(&model)
            .unwrap();
        let action = Tensor::from_slice(&[val], &[1]);
        space.apply(&action, &mut data, &model);
        assert_eq!(data.qfrc_applied[0], f64::from(val));
    }
}
