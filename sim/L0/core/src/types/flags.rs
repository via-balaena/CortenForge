//! Runtime flag helpers for checking disable/enable bitfields.
//!
//! Flag constants live in `enums.rs` (pure values with no dependencies).
//! Helpers that take `&Model` live here, keeping concerns separated.

use super::model::Model;

/// Returns true if the given disable flag is set on the model.
#[inline]
#[must_use]
pub fn disabled(model: &Model, flag: u32) -> bool {
    debug_assert!(
        flag.is_power_of_two() && flag.trailing_zeros() <= 18,
        "disabled() called with non-disable flag: {flag:#x}"
    );
    model.disableflags & flag != 0
}

/// Returns true if the given enable flag is set on the model.
#[inline]
#[must_use]
pub fn enabled(model: &Model, flag: u32) -> bool {
    debug_assert!(
        flag.is_power_of_two() && flag.trailing_zeros() <= 5,
        "enabled() called with non-enable flag: {flag:#x}"
    );
    model.enableflags & flag != 0
}

/// Returns true if actuator `i` is disabled by group membership.
/// Groups outside 0–30 are never disabled (matches MuJoCo).
#[inline]
#[must_use]
pub fn actuator_disabled(model: &Model, i: usize) -> bool {
    let group = model.actuator_group[i];
    // Range check first — only groups 0–30 map to bitmask bits.
    // Cast to u32 only after confirming non-negative.
    if !(0..=30).contains(&group) {
        return false;
    }
    #[allow(clippy::cast_sign_loss)] // group is in 0..=30 (checked above)
    {
        (model.disableactuator & (1u32 << (group as u32))) != 0
    }
}
