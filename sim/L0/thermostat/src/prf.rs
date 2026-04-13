//! `prf.rs` — counter-based hash primitives used across the
//! thermostat crate.
//!
//! Two kinds of primitives live here:
//!
//! 1. **Per-step noise generation.** [`chacha8_block`] +
//!    [`expand_master_seed`] + [`encode_block_counter`] +
//!    [`box_muller_from_block`] form the Route 2 PRF chain that
//!    C-3 stochastic components (starting with
//!    [`LangevinThermostat`](crate::LangevinThermostat)) call
//!    from their `apply` method. These replace the per-component
//!    `Mutex<ChaCha8Rng>` pattern that pre-C-3 stochastic
//!    components used.
//!
//! 2. **Seed derivation.** [`splitmix64`] is the canonical
//!    counter-to-stream hash used by Ch 32's rematch protocol
//!    (and by any other code that needs many uncorrelated `u64`
//!    seeds from one master seed). It is not called by `apply` —
//!    it lives here because it is the same shape (pure
//!    integer-to-pseudorandom function) and belongs in the same
//!    toolbox.
//!
//! See the ML chassis refactor study Ch 15 for the argument that
//! shape 3 + Route 2 is the right chassis primitive, and Ch 32
//! §4.6 for the argument that splitmix64 is the right
//! seed-derivation primitive.

// PR 1a lands this module additively with zero runtime consumers;
// PR 1b (LangevinThermostat rewrite) is the first caller. The
// allow is removed when PR 1b wires the four `pub(crate)`
// functions into `langevin.rs::apply`.
#![allow(dead_code)]

/// Compute one `ChaCha8` block keyed by `key` at block counter
/// `block_counter`. Returns 64 bytes (one `ChaCha` block) of
/// uniform pseudorandom output. Pure function: same inputs
/// always produce the same output.
///
/// The block counter occupies the low 64 bits of `ChaCha`'s
/// 128-bit position field (state words 12 and 13, low-word
/// first); state words 14 and 15 are zero. This matches
/// `rand_chacha 0.9`'s `ChaCha8Rng::set_word_pos(counter * 16)`
/// convention, verified by the cross-check tests in this file.
#[allow(clippy::cast_possible_truncation)]
pub(crate) fn chacha8_block(key: &[u8; 32], block_counter: u64) -> [u8; 64] {
    // ChaCha constants: "expand 32-byte k" as four little-endian u32s.
    const C0: u32 = 0x6170_7865;
    const C1: u32 = 0x3320_646e;
    const C2: u32 = 0x7962_2d32;
    const C3: u32 = 0x6b20_6574;

    let mut state = [0u32; 16];
    state[0] = C0;
    state[1] = C1;
    state[2] = C2;
    state[3] = C3;
    for i in 0..8 {
        state[4 + i] =
            u32::from_le_bytes([key[4 * i], key[4 * i + 1], key[4 * i + 2], key[4 * i + 3]]);
    }
    state[12] = block_counter as u32;
    state[13] = (block_counter >> 32) as u32;
    // state[14] and state[15] stay zero — the high 64 bits of
    // ChaCha's 128-bit position field are unused here.

    let initial = state;

    // 8 rounds = 4 double-rounds. Each double-round runs the
    // quarter-round on the 4 columns, then on the 4 diagonals.
    for _ in 0..4 {
        quarter_round(&mut state, 0, 4, 8, 12);
        quarter_round(&mut state, 1, 5, 9, 13);
        quarter_round(&mut state, 2, 6, 10, 14);
        quarter_round(&mut state, 3, 7, 11, 15);
        quarter_round(&mut state, 0, 5, 10, 15);
        quarter_round(&mut state, 1, 6, 11, 12);
        quarter_round(&mut state, 2, 7, 8, 13);
        quarter_round(&mut state, 3, 4, 9, 14);
    }

    // Feed-forward: add the initial state to the round output.
    for i in 0..16 {
        state[i] = state[i].wrapping_add(initial[i]);
    }

    // Serialize 16 u32s as 64 little-endian bytes.
    let mut out = [0u8; 64];
    for i in 0..16 {
        out[4 * i..4 * i + 4].copy_from_slice(&state[i].to_le_bytes());
    }
    out
}

const fn quarter_round(state: &mut [u32; 16], a: usize, b: usize, c: usize, d: usize) {
    state[a] = state[a].wrapping_add(state[b]);
    state[d] = (state[d] ^ state[a]).rotate_left(16);
    state[c] = state[c].wrapping_add(state[d]);
    state[b] = (state[b] ^ state[c]).rotate_left(12);
    state[a] = state[a].wrapping_add(state[b]);
    state[d] = (state[d] ^ state[a]).rotate_left(8);
    state[c] = state[c].wrapping_add(state[d]);
    state[b] = (state[b] ^ state[c]).rotate_left(7);
}

/// Expand a `u64` master seed into a 32-byte `ChaCha8` key using
/// the same rule `rand_chacha 0.9`'s
/// `SeedableRng::seed_from_u64` uses internally (the `rand_core`
/// default: PCG32-XSH-RR fill over eight u32 chunks).
///
/// The cross-verification tests in this module assert that the
/// end-to-end pipeline
/// `chacha8_block(&expand_master_seed(s), n)` matches
/// `ChaCha8Rng::seed_from_u64(s)` advanced to block `n`.
#[allow(clippy::cast_possible_truncation)]
pub(crate) fn expand_master_seed(master_seed: u64) -> [u8; 32] {
    // PCG32-XSH-RR constants, matching rand_core 0.9's default
    // seed_from_u64 implementation.
    const MUL: u64 = 6_364_136_223_846_793_005;
    const INC: u64 = 11_634_580_027_462_260_723;

    let mut state = master_seed;
    let mut seed = [0u8; 32];
    for chunk in seed.chunks_exact_mut(4) {
        state = state.wrapping_mul(MUL).wrapping_add(INC);
        let xorshifted = (((state >> 18) ^ state) >> 27) as u32;
        let rot = (state >> 59) as u32;
        let word = xorshifted.rotate_right(rot);
        chunk.copy_from_slice(&word.to_le_bytes());
    }
    seed
}

/// Encode a `(traj_id, step_index)` pair into a 64-bit block
/// counter. 32/32 split (D4): `traj_id` occupies the high 32
/// bits, `step_index` the low 32 bits.
pub(crate) const fn encode_block_counter(traj_id: u64, step_index: u64) -> u64 {
    (traj_id << 32) | (step_index & 0xFFFF_FFFF)
}

/// Apply Box–Muller to a 64-byte block, producing 8 `f64`
/// Gaussians. The block is consumed in four 16-byte chunks; each
/// chunk's two little-endian `u64`s become `(u1, u2)`, mapped to
/// open-interval uniforms `(0, 1]`, then transformed into a
/// Gaussian pair `(r cos θ, r sin θ)` with `r = sqrt(-2 ln u1)`
/// and `θ = 2π u2`.
pub(crate) fn box_muller_from_block(block: &[u8; 64]) -> [f64; 8] {
    let mut out = [0.0f64; 8];
    for pair in 0..4 {
        let base = pair * 16;
        let u1_bits = read_u64_le(block, base);
        let u2_bits = read_u64_le(block, base + 8);
        let u1 = u64_to_uniform_open(u1_bits);
        let u2 = u64_to_uniform_open(u2_bits);
        let r = (-2.0 * u1.ln()).sqrt();
        let theta = std::f64::consts::TAU * u2;
        out[2 * pair] = r * theta.cos();
        out[2 * pair + 1] = r * theta.sin();
    }
    out
}

const fn read_u64_le(block: &[u8; 64], offset: usize) -> u64 {
    u64::from_le_bytes([
        block[offset],
        block[offset + 1],
        block[offset + 2],
        block[offset + 3],
        block[offset + 4],
        block[offset + 5],
        block[offset + 6],
        block[offset + 7],
    ])
}

#[allow(clippy::cast_precision_loss)]
fn u64_to_uniform_open(bits: u64) -> f64 {
    // Map 64 random bits to a uniform in (0, 1]. Take the top 53
    // bits as a mantissa, add 1 to shift off zero, divide by 2^53.
    ((bits >> 11).wrapping_add(1)) as f64 * (1.0 / ((1u64 << 53) as f64))
}

/// `SplitMix64` — the canonical counter-to-stream hash for
/// deriving many uncorrelated `u64` seeds from one master.
///
/// Formula from Sebastiano Vigna's public-domain reference
/// implementation at <https://prng.di.unimi.it/splitmix64.c>.
#[must_use]
pub const fn splitmix64(seed: u64) -> u64 {
    let mut z = seed.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E7B5);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    use rand::{RngCore, SeedableRng};
    use rand_chacha::ChaCha8Rng;

    fn ref_block(seed: u64, counter: u64) -> [u8; 64] {
        let mut rng = ChaCha8Rng::seed_from_u64(seed);
        rng.set_word_pos(u128::from(counter) * 16);
        let mut out = [0u8; 64];
        rng.fill_bytes(&mut out);
        out
    }

    fn our_block(seed: u64, counter: u64) -> [u8; 64] {
        chacha8_block(&expand_master_seed(seed), counter)
    }

    #[test]
    fn splitmix64_matches_vigna_formula_at_zero() {
        // Pinned output of Vigna's splitmix64 formula at seed=0,
        // independently verified against a Python transcription
        // of the reference C implementation at
        // <https://prng.di.unimi.it/splitmix64.c>.
        assert_eq!(splitmix64(0), 0x8B57_DAFC_A0CE_E644);
    }

    #[test]
    fn splitmix64_matches_inline_formula() {
        // Regression guard: a private inline copy of Vigna's
        // formula. If the production `splitmix64` ever drifts
        // from this shape, the assert fires. Combined with the
        // external anchor at seed=0, this pins the formula at
        // multiple output points.
        fn reference(seed: u64) -> u64 {
            let mut z = seed.wrapping_add(0x9E37_79B9_7F4A_7C15);
            z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E7B5);
            z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            z ^ (z >> 31)
        }
        for seed in [1u64, 42, 0xCAFE_BABE, u64::MAX] {
            assert_eq!(splitmix64(seed), reference(seed));
        }
    }

    #[test]
    fn encode_block_counter_round_trip() {
        let cases = [
            (0u64, 0u64),
            (1, 0),
            (0, 1),
            (42, 100),
            (0xFFFF_FFFF, 0xFFFF_FFFF),
        ];
        for (traj_id, step_index) in cases {
            let encoded = encode_block_counter(traj_id, step_index);
            assert_eq!(encoded >> 32, traj_id);
            assert_eq!(encoded & 0xFFFF_FFFF, step_index);
        }
    }

    #[test]
    fn chacha8_block_matches_rand_chacha_at_origin() {
        assert_eq!(our_block(0, 0), ref_block(0, 0));
    }

    #[test]
    fn chacha8_block_matches_rand_chacha_at_block_boundary_and_crossover() {
        // Brackets the state[12] u32 wraparound: counter at the
        // u32 maximum (last block where state[13] is zero) and
        // counter at u32_max + 1 (first block where state[13]
        // is nonzero).
        assert_eq!(our_block(42, 0xFFFF_FFFF), ref_block(42, 0xFFFF_FFFF));
        assert_eq!(our_block(42, 0x1_0000_0000), ref_block(42, 0x1_0000_0000));
    }

    #[test]
    fn chacha8_block_matches_rand_chacha_near_u64_max() {
        let counter = u64::MAX - 1;
        assert_eq!(our_block(u64::MAX, counter), ref_block(u64::MAX, counter));
    }

    #[test]
    fn chacha8_block_matches_rand_chacha_at_chosen_pairs() {
        // Three (seed, counter) pairs pinned at authoring time
        // to exercise seeds that touch all eight bytes of a u64
        // and counters that span multiple bit widths.
        let pairs: [(u64, u64); 3] = [
            (0x0123_4567_89AB_CDEF, 0x0000_0000_DEAD_BEEF),
            (0xCAFE_BABE_DEAD_BEEF, 1_000_000),
            (7, 0x0000_0001_0000_0042),
        ];
        for (seed, counter) in pairs {
            assert_eq!(
                our_block(seed, counter),
                ref_block(seed, counter),
                "mismatch at (seed={seed:#x}, counter={counter:#x})"
            );
        }
    }

    #[test]
    fn chacha8_block_is_pure() {
        let key = expand_master_seed(12345);
        let a = chacha8_block(&key, 42);
        let b = chacha8_block(&key, 42);
        assert_eq!(a, b);
    }

    #[test]
    fn expand_master_seed_differs_for_different_seeds() {
        let a = expand_master_seed(0);
        let b = expand_master_seed(1);
        let c = expand_master_seed(u64::MAX);
        assert_ne!(a, b);
        assert_ne!(a, c);
        assert_ne!(b, c);
    }

    #[test]
    fn box_muller_from_zero_block_is_finite() {
        let zeros = [0u8; 64];
        let out = box_muller_from_block(&zeros);
        for &v in &out {
            assert!(v.is_finite(), "box_muller(zeros) produced non-finite: {v}");
        }
        // All four pairs share inputs (u1, u2) = (2^-53, 2^-53),
        // so the cos-components at even indices are equal and
        // the sin-components at odd indices are equal.
        assert_eq!(out[0], out[2]);
        assert_eq!(out[0], out[4]);
        assert_eq!(out[0], out[6]);
        assert_eq!(out[1], out[3]);
        assert_eq!(out[1], out[5]);
        assert_eq!(out[1], out[7]);
        // With u1 = u2 = 2^-53, r = sqrt(-2 ln 2^-53) ≈ 8.57 and
        // θ ≈ 2π · 2^-53 ≈ 7e-16, so cos θ ≈ 1 and sin θ is
        // near machine epsilon.
        assert!(out[0].abs() > 1.0);
        assert!(out[1].abs() < 1e-10);
    }
}
