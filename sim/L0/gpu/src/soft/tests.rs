//! The soft executor's gates (recon §17b): per-phase conformance against the
//! CPU executor at f32 on every fixture, and the gates on what is particular
//! to the GPU. Each gate was made to fail once by the change §17b names.

#![cfg(test)]
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::many_single_char_names,
    clippy::unwrap_used
)]

mod conformance;
mod fixtures;
mod gates;

use sim_soft_explicit::executor::{Monitors, PhaseOutputs, Snapshot};

use crate::context::GpuContext;
use crate::test_support::gpu_context_or_skip;

fn context() -> Option<GpuContext> {
    gpu_context_or_skip("soft executor")
}

/// The largest difference between two arrays of one length, over the larger
/// of their largest magnitudes; the difference itself where both are zero.
fn relative_difference(a: &[f64], b: &[f64]) -> f64 {
    assert_eq!(a.len(), b.len(), "arrays of different lengths");
    let (mut difference, mut largest) = (0.0_f64, 0.0_f64);
    for (x, y) in a.iter().zip(b) {
        difference = difference.max((x - y).abs());
        largest = largest.max(x.abs()).max(y.abs());
    }
    if largest == 0.0 {
        difference
    } else {
        difference / largest
    }
}

/// Every value a snapshot holds, by its bits: a `NaN` by its payload, and
/// `-0.0` apart from `0.0`.
fn snapshot_bits(s: &Snapshot) -> Vec<u64> {
    [
        &s.displacements,
        &s.velocities,
        &s.anchors,
        &s.displacement_sums,
        &s.friction_sums,
    ]
    .into_iter()
    .flatten()
    .flatten()
    .chain(&s.normal_force_sums)
    .map(|x| x.to_bits())
    .chain([s.accumulated_steps])
    .collect()
}

/// Every phase output, by its bits.
fn outputs_bits(o: &PhaseOutputs) -> Vec<u64> {
    [
        &o.dilations,
        &o.volume_changes,
        &o.pressures,
        &o.normal_forces,
    ]
    .into_iter()
    .flatten()
    .chain(o.element_forces.iter().flatten())
    .chain(o.element_viscous_forces.iter().flatten())
    .chain(
        [&o.elastic_forces, &o.viscous_forces, &o.contact_forces]
            .into_iter()
            .flatten()
            .flatten(),
    )
    .map(|x| x.to_bits())
    .collect()
}

/// Every monitor, by its bits.
fn monitor_bits(m: &Monitors) -> Vec<u64> {
    [
        m.kinetic_energy,
        m.internal_energy,
        m.contact_kinetic_energy,
        m.normal_force,
        m.contact_work,
        m.obstacle_work,
        m.damping_loss,
        m.max_penetration,
        m.deepest_prediction,
    ]
    .iter()
    .chain(&m.contact_force)
    .chain(&m.contact_moment)
    .map(|x| x.to_bits())
    .chain([m.steps, m.inverted_element_steps, m.coarse_corrections])
    .collect()
}
