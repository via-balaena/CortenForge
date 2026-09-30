//! The soft executor's gates (recon §17b): per-phase conformance against the
//! CPU executor at f32 on every fixture, and the gates on what is particular
//! to the GPU. Each gate was made to fail once by the change §17b names.

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

use crate::context::GpuContext;
use crate::test_support::gpu_context_or_skip;

fn context() -> Option<GpuContext> {
    gpu_context_or_skip("soft executor")
}

/// The largest difference between two arrays, over the larger of their
/// largest magnitudes; the difference itself where both are zero.
fn relative_difference<'a>(
    a: impl IntoIterator<Item = &'a f64>,
    b: impl IntoIterator<Item = &'a f64>,
) -> f64 {
    let (mut difference, mut largest) = (0.0_f64, 0.0_f64);
    for (x, y) in a.into_iter().zip(b) {
        difference = difference.max((x - y).abs());
        largest = largest.max(x.abs()).max(y.abs());
    }
    if largest == 0.0 {
        difference
    } else {
        difference / largest
    }
}
