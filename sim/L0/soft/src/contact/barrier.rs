//! The IPC log-barrier `b(d, d̂)` and the stiffness `κ` that scales it.
//!
//! One implementation of the barrier, shared by everything that needs it. Until
//! this module existed the formula appeared three times in `sim-soft` alone —
//! once in [`IpcRigidContact`](super::IpcRigidContact)'s solver path and twice
//! in [`face`](super::face)'s test helpers — with the solver's copy using an
//! FMA (`mul_add`) the others did not, so the copies already differed in their
//! last bits. A barrier that is re-typed per consumer is a barrier that can
//! drift, and the derived-κ work below depends on the design arithmetic being
//! *the same function* the solver evaluates, not a re-derivation of it.
//!
//! # The barrier
//!
//! ```text
//!     b(d, d̂) = -(d - d̂)² ln(d / d̂)     for 0 < d < d̂,   0 for d ≥ d̂
//! ```
//!
//! with `b(d̂) = b'(d̂) = b''(d̂) = 0`, which is what makes it `C²` across the
//! active-set boundary. Derived in the book at
//! `docs/studies/soft_body_architecture/src/40-contact/01-ipc-internals/00-barrier.md`.
//!
//! # κ, and why it is a traction here
//!
//! The **per-vertex** barrier applies `κ·b` as a point energy, so `κ` carries
//! units of `J/m² = N/m`. The **surface-integrated face** barrier (rung 8b)
//! applies it as an energy *density over the contact face*, weighted by the
//! face's rest area:
//!
//! ```text
//!     E = A_rest · Σ_q ŵ_q · κ · b(sd(x_q)),     Σ_q ŵ_q = 1
//! ```
//!
//! so on the face path `κ` carries units of `J/m⁴ = Pa/m`, and the quantity
//! `κ·|b'(d)|` is a **traction in pascals**.
//!
//! # ⛔ There are THREE κ in this crate and they are not comparable numbers
//!
//! ```text
//!   contact model            energy per pair              units of κ
//!   ──────────────────────────────────────────────────────────────────
//!   IPC, per-vertex          κ · b(sd)                    J/m² = N/m
//!   IPC, per-FACE (here)     A_rest · Σ_q ŵ_q · κ · b(sd) J/m⁴ = Pa/m
//!   Penalty, per-vertex      ½ · κ · gap²                 J/m² = N/m
//! ```
//!
//! ⚠⚠ **The face κ differs from the other two by an AREA.** A value carried
//! between paths is not merely re-tuned, it is *re-interpreted* — comparing
//! their magnitudes is a category error, not a calibration observation.
//!
//! ⭐ **This matters concretely for the `insertion_sim` renovation.** That tool
//! runs [`PenaltyRigidContact`](super::PenaltyRigidContact) today at
//! `INSERTION_CONTACT_KAPPA = 1e3` (`tools/cf-sim-research/src/insertion_sim.rs`),
//! and the queued bridge moves it onto this face path, where the derived value
//! is `1e7`. That is **four orders of a different quantity**, not a 10 000×
//! stiffening, and nothing in either number says so.
//!
//! ⚠ **A κ carried between fixtures on the SAME path is a different mistake,
//! and it has also already happened here.** Rung 8b's face κ of `1e4` moved
//! into a 1.2 mm-band fixture is dimensionally fine and simply wrong by scale
//! — it made the Tet10 × Yeoh ramp look like it walled at 1 % strain when it
//! reaches 44 %. Units and scale fail independently; neither implies the
//! other.
//!
//! [`face_barrier_standoff`] and [`face_barrier_kappa`] are the two directions
//! of that traction relation, and are the arithmetic behind a κ chosen by
//! derivation rather than by sweeping decades until one converges.

/// Floor applied to the gap before evaluating the barrier, as a fraction of
/// `d̂`.
///
/// The barrier is defined on `0 < d < d̂`; a *penetrating* trial iterate has
/// `sd ≤ 0`, where `ln(d/d̂)` is `NaN`. Flooring yields a finite, strongly
/// repulsive value instead, so a Newton line search that overshoots gets a
/// large number to back off from rather than a `NaN` that poisons the residual.
///
/// ⚠ This is why a converged solve can report `min_sd ≤ 0` at a tiny residual:
/// the clamp makes the energy finite where the true barrier is infinite. A
/// non-penetration claim must read the signed distance, not the residual.
pub const BARRIER_GAP_FLOOR_FRACTION: f64 = 1.0e-6;

/// The gap the barrier is evaluated at: `sd` floored at `d̂` times
/// [`BARRIER_GAP_FLOOR_FRACTION`].
///
/// `pub(crate)`: an internal detail of how the three barrier functions clamp,
/// with no caller outside this crate. [`BARRIER_GAP_FLOOR_FRACTION`] *is*
/// public, because it explains an observable — a converged solve reporting
/// `min_sd ≤ 0` — which a consumer reading a contact readout needs.
#[must_use]
#[inline]
pub(crate) fn barrier_gap(sd: f64, d_hat: f64) -> f64 {
    sd.max(d_hat * BARRIER_GAP_FLOOR_FRACTION)
}

/// `b(d, d̂) = -(d - d̂)² ln(d / d̂)` — zero outside the support.
#[must_use]
pub fn barrier_value(sd: f64, d_hat: f64) -> f64 {
    if sd >= d_hat {
        return 0.0;
    }
    let d = barrier_gap(sd, d_hat);
    let r = d - d_hat;
    let ln = (d / d_hat).ln();
    -(r * r) * ln
}

/// `b'(d, d̂) = -2(d - d̂) ln(d/d̂) - (d - d̂)² / d` — zero outside the support.
///
/// Negative on the active band (the barrier pushes outward), and `κ·|b'|` is
/// the face barrier's traction in pascals.
#[must_use]
pub fn barrier_derivative(sd: f64, d_hat: f64) -> f64 {
    if sd >= d_hat {
        return 0.0;
    }
    let d = barrier_gap(sd, d_hat);
    let r = d - d_hat;
    let ln = (d / d_hat).ln();
    r.mul_add(-2.0 * ln, -(r * r) / d)
}

/// `b''(d, d̂) = -2 ln(d/d̂) - 4(d - d̂)/d + (d - d̂)²/d²` — zero outside the
/// support.
#[must_use]
pub fn barrier_second_derivative(sd: f64, d_hat: f64) -> f64 {
    if sd >= d_hat {
        return 0.0;
    }
    let d = barrier_gap(sd, d_hat);
    let r = d - d_hat;
    let ln = (d / d_hat).ln();
    // Written in exactly the operation order `IpcRigidContact::barrier` used
    // before this module existed, FMA included where it had one and excluded
    // where it did not, so extracting it is bit-identical rather than merely
    // algebraically equal. `barrier_derivative` keeps the `mul_add`; this one
    // must not acquire one.
    r * r / (d * d) - 4.0 * r / d - 2.0 * ln
}

/// Traction (Pa) the surface-integrated face barrier exerts at gap `sd`.
///
/// `κ·|b'(sd)|`. This is the quantity that balances the elastic contact stress
/// at equilibrium, and the one the two functions below invert.
#[must_use]
pub fn face_barrier_traction(kappa: f64, sd: f64, d_hat: f64) -> f64 {
    kappa * barrier_derivative(sd, d_hat).abs()
}

/// The gap at which a face barrier of stiffness `kappa` balances
/// `traction_pa` — the standoff the barrier holds under that load.
///
/// `|b'|` is strictly decreasing on `(0, d̂)`, diverging as `d → 0⁺` and
/// vanishing at `d̂`, so the balance `κ·|b'(d)| = σ` has exactly one root there
/// for any `σ > 0`. Found by bisection on the real [`barrier_derivative`], so
/// it inverts the shipped barrier rather than a closed form fitted to it.
///
/// Returns `d̂` when nothing is pushing — a non-positive or non-finite
/// `traction_pa`, or a non-positive `kappa` — since an unengaged barrier holds
/// the whole band. Returns the clamp floor when not even that gap could carry
/// the load, so the result always lies between `d̂` times
/// [`BARRIER_GAP_FLOOR_FRACTION`] and `d̂`, and is never `NaN`.
#[must_use]
pub fn face_barrier_standoff(kappa: f64, d_hat: f64, traction_pa: f64) -> f64 {
    if !(traction_pa.is_finite() && traction_pa > 0.0 && kappa > 0.0) {
        return d_hat;
    }
    let mut lo = d_hat * BARRIER_GAP_FLOOR_FRACTION;
    let mut hi = d_hat;
    if face_barrier_traction(kappa, lo, d_hat) < traction_pa {
        return lo; // even the clamp floor cannot carry this load
    }
    // 200 bisections on a bracket of width d̂ is far past f64 resolution; the
    // loop is bounded rather than tolerance-driven so it cannot spin.
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if mid <= lo || mid >= hi {
            break;
        }
        if face_barrier_traction(kappa, mid, d_hat) > traction_pa {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

/// The face-barrier stiffness that holds `standoff` against `traction_pa`.
///
/// The inverse of [`face_barrier_standoff`], in closed form:
/// `κ = σ / |b'(standoff)|`. **This is the derivation** — given the traction a
/// scene must carry and the gap the barrier must keep open under it, κ is
/// determined, not swept.
///
/// Returns `None` when `standoff` is not strictly inside the open band
/// `(0, d̂)`: at or beyond `d̂` the barrier is inactive (`b' = 0`) and no finite
/// stiffness holds a gap it cannot see. A `None` here is a mis-specified
/// requirement, and is returned rather than an infinity precisely so it cannot
/// be carried forward as a number.
#[must_use]
pub fn face_barrier_kappa(d_hat: f64, standoff: f64, traction_pa: f64) -> Option<f64> {
    let inside_band = standoff > 0.0 && standoff < d_hat;
    let loadable = traction_pa.is_finite() && traction_pa >= 0.0;
    if !inside_band || !loadable {
        return None;
    }
    let b_prime = barrier_derivative(standoff, d_hat).abs();
    if b_prime <= 0.0 {
        return None;
    }
    Some(traction_pa / b_prime)
}

#[cfg(test)]
#[allow(clippy::expect_used)]
mod tests {
    use super::*;

    /// Gaps well inside the band, so the `d → 0` clamp never participates.
    const PROBE_GAPS: [f64; 7] = [0.05, 0.15, 0.3, 0.5, 0.7, 0.9, 0.97];
    const D_HAT: f64 = 1.2e-3;

    /// The book's closed forms, typed from
    /// `docs/studies/soft_body_architecture/src/40-contact/01-ipc-internals/00-barrier.md`
    /// in **its** algebra, not the shipped one's.
    ///
    /// The book writes `b'` factored as `(d̂ - d)[2 ln(d/d̂) - d̂/d + 1]` and
    /// `b''` as `-2 ln(d/d̂) + (d̂/d)(d̂/d + 2) - 3`. The shipped code writes
    /// them expanded, with an FMA. Same functions, different expressions — so
    /// agreement between them is evidence about the *formula*, which a test
    /// that evaluated the shipped code on both sides could never give.
    fn book_b(d: f64, d_hat: f64) -> f64 {
        let g = d_hat - d;
        -(g * g) * (d / d_hat).ln()
    }
    fn book_b_prime(d: f64, d_hat: f64) -> f64 {
        (d_hat - d) * (2.0 * (d / d_hat).ln() - d_hat / d + 1.0)
    }
    fn book_b_second(d: f64, d_hat: f64) -> f64 {
        let q = d_hat / d;
        -2.0 * (d / d_hat).ln() + q * (q + 2.0) - 3.0
    }

    #[test]
    fn derivatives_match_the_books_closed_forms() {
        for f in PROBE_GAPS {
            let d = f * D_HAT;
            let (got_0, want_0) = (barrier_value(d, D_HAT), book_b(d, D_HAT));
            let rel_0 = (got_0 - want_0).abs() / want_0.abs().max(f64::MIN_POSITIVE);
            assert!(
                rel_0 < 1e-12,
                "b at d/d_hat = {f}: shipped {got_0:e}, book {want_0:e} (rel {rel_0:e})",
            );
            let (got_1, want_1) = (barrier_derivative(d, D_HAT), book_b_prime(d, D_HAT));
            let (got_2, want_2) = (barrier_second_derivative(d, D_HAT), book_b_second(d, D_HAT));
            let rel_1 = (got_1 - want_1).abs() / want_1.abs().max(f64::MIN_POSITIVE);
            let rel_2 = (got_2 - want_2).abs() / want_2.abs().max(f64::MIN_POSITIVE);
            assert!(
                rel_1 < 1e-12,
                "b' at d/d_hat = {f}: shipped {got_1:e}, book {want_1:e} (rel {rel_1:e})",
            );
            assert!(
                rel_2 < 1e-12,
                "b'' at d/d_hat = {f}: shipped {got_2:e}, book {want_2:e} (rel {rel_2:e})",
            );
        }
    }

    /// Central differences on the shipped `b` — independent of anyone's
    /// algebra, so it catches an error the book and the code could share.
    #[test]
    fn derivatives_match_finite_differences() {
        for f in PROBE_GAPS {
            let d = f * D_HAT;
            let h = 1e-7 * D_HAT;
            let fd_1 = (barrier_value(d + h, D_HAT) - barrier_value(d - h, D_HAT)) / (2.0 * h);
            let fd_2 =
                (barrier_derivative(d + h, D_HAT) - barrier_derivative(d - h, D_HAT)) / (2.0 * h);
            let b_1 = barrier_derivative(d, D_HAT);
            let b_2 = barrier_second_derivative(d, D_HAT);
            let rel_1 = (fd_1 - b_1).abs() / b_1.abs().max(f64::MIN_POSITIVE);
            let rel_2 = (fd_2 - b_2).abs() / b_2.abs().max(f64::MIN_POSITIVE);
            assert!(
                rel_1 < 1e-5,
                "b' at d/d_hat = {f}: analytic {b_1:e}, FD of b {fd_1:e} (rel {rel_1:e})",
            );
            assert!(
                rel_2 < 1e-5,
                "b'' at d/d_hat = {f}: analytic {b_2:e}, FD of b' {fd_2:e} (rel {rel_2:e})",
            );
        }
    }

    /// `b(d̂) = b'(d̂) = b''(d̂) = 0` — the `C²` truncation the whole active-set
    /// machinery rests on (book Ch 40.01.00, "Boundary conditions").
    ///
    /// Asserted as *rates*, not as "small". Writing `d = d̂(1 - ε)` and
    /// expanding, the three vanish as
    ///
    /// ```text
    ///     |b| -> 1 e³ d̂²,    |b'| -> 3 e² d̂,    |b''| -> 6 e
    /// ```
    ///
    /// and those coefficients are measured to converge to exactly 1, 3 and 6
    /// (to 4 decimals by ε = 1e-6). A tolerance on the *value* would pass for
    /// any function that merely got small near the edge; a tolerance on the
    /// *coefficient* pins the order of contact, which is the property `C²`
    /// actually names.
    #[test]
    fn the_barrier_is_c2_at_the_band_edge() {
        for eps in [1e-9, 1e-6, 1e-4, 1e-3] {
            let d = D_HAT * (1.0 - eps);
            let rows = [
                (
                    "b",
                    barrier_value(d, D_HAT).abs() / (eps.powi(3) * D_HAT * D_HAT),
                    1.0,
                ),
                (
                    "b'",
                    barrier_derivative(d, D_HAT).abs() / (eps * eps * D_HAT),
                    3.0,
                ),
                ("b''", barrier_second_derivative(d, D_HAT).abs() / eps, 6.0),
            ];
            for (name, coeff, want) in rows {
                let rel = (coeff - want).abs() / want;
                assert!(
                    rel < 1e-2,
                    "{name} must vanish at d_hat like {want} * eps^n; at eps = {eps:e} \
                     its coefficient is {coeff} (rel {rel:e}), so the barrier does not \
                     make C2 contact with the zero outside the band",
                );
            }
        }
        // Outside the support the barrier is *defined* to be exactly zero —
        // the compact support the active-set cull depends on. Bit equality is
        // the claim here, not a tolerance that happened to be tight.
        #[allow(clippy::float_cmp)]
        for d in [D_HAT, D_HAT * 1.5, D_HAT * 100.0] {
            assert_eq!(barrier_value(d, D_HAT), 0.0, "b must vanish at {d:e}");
            assert_eq!(barrier_derivative(d, D_HAT), 0.0, "b' must vanish at {d:e}");
            assert_eq!(
                barrier_second_derivative(d, D_HAT),
                0.0,
                "b'' must vanish at {d:e}",
            );
        }
    }

    /// `|b'|` is strictly decreasing on the band — the monotonicity
    /// [`face_barrier_standoff`]'s bisection relies on for a unique root.
    #[test]
    fn the_traction_is_strictly_decreasing_across_the_band() {
        let mut prev = f64::INFINITY;
        for i in 1..=999 {
            let d = D_HAT * f64::from(i) / 1000.0;
            let t = barrier_derivative(d, D_HAT).abs();
            assert!(
                t < prev,
                "|b'| rose at d/d_hat = {}: {t:e} after {prev:e} — the standoff \
                 inversion assumes a single crossing and would be unsound",
                f64::from(i) / 1000.0,
            );
            prev = t;
        }
    }

    /// The two directions of the traction relation must invert each other.
    #[test]
    fn standoff_and_kappa_are_inverses() {
        for f in PROBE_GAPS {
            let standoff = f * D_HAT;
            for traction in [1.0e2, 1.0e3, 1.0e4, 4.0e4, 1.0e5] {
                let kappa = face_barrier_kappa(D_HAT, standoff, traction)
                    .expect("a standoff inside the band has a finite kappa");
                let back = face_barrier_standoff(kappa, D_HAT, traction);
                let rel = (back - standoff).abs() / standoff;
                assert!(
                    rel < 1e-9,
                    "kappa {kappa:e} derived to hold {standoff:e} m against \
                     {traction:e} Pa holds {back:e} m instead (rel {rel:e})",
                );
                // ⛔ An arm asserting `face_barrier_traction(kappa, standoff)
                // == traction` was removed here: it evaluates
                // `(sigma / |b'|) * |b'|`, so `barrier_derivative` cancels and
                // it cannot detect an error in it. The arm above is the real
                // content — it exercises the BISECTION, which has to recover
                // the root from a stiffness, and would fail if `|b'|` were not
                // monotone on the band.
            }
        }
    }

    /// An unengaged barrier holds the whole band, and a load past anything it
    /// can carry pins at the clamp floor. Both edges return a real gap rather
    /// than `NaN`, because the caller's next step is arithmetic on it.
    /// Bit equality is the claim at both edges: an unengaged barrier returns
    /// `d̂` itself and an over-loaded one returns the floor itself, not a value
    /// near them.
    #[test]
    #[allow(clippy::float_cmp)]
    fn the_standoff_edges_return_a_gap_not_a_nan() {
        for dead in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert_eq!(
                face_barrier_standoff(1.0e7, D_HAT, dead),
                D_HAT,
                "traction {dead:?} means nothing is pushing, so the barrier                  holds its whole band",
            );
        }
        for dead_kappa in [0.0, -1.0] {
            assert_eq!(
                face_barrier_standoff(dead_kappa, D_HAT, 4.0e4),
                D_HAT,
                "kappa {dead_kappa} is not a barrier at all",
            );
        }
        // Past what even the clamp floor can carry. The floor gap is
        // d_hat * 1e-6, where |b'| is ~1e3, so 1e7 * that is ~1e10 Pa.
        let crushing = face_barrier_standoff(1.0e7, D_HAT, 1.0e15);
        assert_eq!(
            crushing,
            D_HAT * BARRIER_GAP_FLOOR_FRACTION,
            "a load past the clamped barrier's capacity must pin at the floor",
        );
        assert!(crushing.is_finite() && crushing > 0.0);
    }

    /// A requirement the barrier cannot meet must come back as `None`, never as
    /// an infinity that a caller would carry forward as a number.
    #[test]
    fn an_unmeetable_standoff_has_no_kappa() {
        for bad in [D_HAT, D_HAT * 1.001, D_HAT * 10.0, 0.0, -1.0e-6] {
            assert_eq!(
                face_barrier_kappa(D_HAT, bad, 4.0e4),
                None,
                "a standoff of {bad:e} m is not strictly inside (0, {D_HAT:e}) \
                 and must not yield a kappa",
            );
        }
        assert!(
            face_barrier_kappa(D_HAT, 0.3 * D_HAT, f64::NAN).is_none(),
            "a non-finite traction must not yield a kappa",
        );
    }
}

#[cfg(test)]
mod bit_identity {
    use super::*;

    /// The extracted barrier, held against the expressions
    /// `IpcRigidContact::barrier` carried at `3aaa295c`, **bit for bit**.
    ///
    /// The three bodies below were lifted mechanically out of that commit's
    /// `ipc.rs` (the only edit: `self.d_hat` -> the local `d_hat`), so this is
    /// a comparison against history rather than against anyone's recollection
    /// of it.
    ///
    /// This is what licenses the claim that extracting the barrier into this
    /// module changed no solver result — every converged pose, gate number and
    /// pinned residual in the tree was measured against those expressions.
    ///
    /// ⚠ **Algebraic equality would not be enough, and that is the whole point
    /// of comparing bits.** Two rewrites that are exactly the same function on
    /// paper are caught here:
    /// - **`b'` unfused.** `r.mul_add(-2·ln, -(r²)/d)` rounds once;
    ///   `-2·r·ln - (r²)/d` rounds twice. Measured to first differ at
    ///   `sd = 4.86e-6, d̂ = 1.2e-3`.
    /// - **`b''` regrouped.** `a - b - c` versus `a - (b + c)`. Measured to
    ///   first differ at `sd = 8.4e-8` — an order tighter, because the three
    ///   terms only fail to cancel identically very close to the clamp.
    ///
    /// ⚠ **`b''` is *not* sensitive to an added FMA**, which an earlier draft
    /// of this module asserted that it was: `2.0 * ln` is exact in binary
    /// floating point, so fusing that product saves no rounding. Association
    /// is what matters there, fusion is what matters in `b'`, and only
    /// measurement separated the two.
    #[test]
    // `unused_parens`: the three expressions below are pasted from the old
    // source, where each sat inside `self.kappa * (...)`. Stripping the
    // parentheses to satisfy the lint would mean editing the very text this
    // test exists to compare against, so they stay.
    // `cast_precision_loss`: the u64 -> f64 casts are a seeded xorshift being
    // turned into a scatter of gaps; losing low bits is what a scatter wants.
    #[allow(unused_parens, clippy::cast_precision_loss)]
    fn the_extraction_matches_the_expressions_ipc_carried_at_3aaa295c() {
        let mut checked = 0_u64;
        let mut penetrating = 0_u64;
        let mut near_edge = 0_u64;

        // Band widths actually used in the tree: the Yeoh fixture's 1.2 mm,
        // rung 8b's 10 mm, its sphere arm's 50 mm, and a unit band.
        for &d_hat in &[1.2e-3_f64, 1.0e-2, 5.0e-2, 1.0] {
            let mut gaps: Vec<f64> = Vec::new();
            // (a) uniform grid, including penetrating gaps that exercise the clamp
            for i in -5_000..200_000_i32 {
                gaps.push(d_hat * f64::from(i) / 100_000.0);
            }
            // (b) logarithmic approach to zero, where the clamp and the log bite
            for k in 0..400 {
                gaps.push(d_hat * 10f64.powf(-12.0 + f64::from(k) / 25.0));
            }
            // (c) ulp-scale neighbourhood of the band edge, where b, b', b''
            //     all vanish and cancellation is worst
            for k in 0..2_000 {
                let e = d_hat * (1.0 - f64::from(k) * 1e-12);
                gaps.push(e);
                gaps.push(d_hat * (1.0 + f64::from(k) * 1e-12));
            }
            // (d) a deterministic scatter, so nothing depends on grid alignment
            let mut x = 0x2545_F491_4F6C_DD1D_u64 ^ d_hat.to_bits();
            for _ in 0..50_000 {
                x ^= x << 13;
                x ^= x >> 7;
                x ^= x << 17;
                let u = (x >> 11) as f64 / (1_u64 << 53) as f64;
                gaps.push(d_hat * (u * 1.5 - 0.25));
            }

            for sd in gaps {
                if sd >= d_hat {
                    // Outside the support both sides are defined to be zero;
                    // the old code returned `None` here and never evaluated.
                    continue;
                }
                if sd <= 0.0 {
                    penetrating += 1;
                }
                if (sd / d_hat - 1.0).abs() < 1e-9 {
                    near_edge += 1;
                }

                // ---- verbatim from 3aaa295c ----
                let d = sd.max(d_hat * 1.0e-6);
                let r = d - d_hat;
                let ln = (d / d_hat).ln();
                let old_energy = (-(r * r) * ln);
                let old_d1 = r.mul_add(-2.0 * ln, -(r * r) / d);
                let old_d2 = (r * r / (d * d) - 4.0 * r / d - 2.0 * ln);
                // --------------------------------

                assert_eq!(
                    barrier_value(sd, d_hat).to_bits(),
                    old_energy.to_bits(),
                    "b differs at sd = {sd:e}, d_hat = {d_hat:e}",
                );
                assert_eq!(
                    barrier_derivative(sd, d_hat).to_bits(),
                    old_d1.to_bits(),
                    "b' differs at sd = {sd:e}, d_hat = {d_hat:e}",
                );
                assert_eq!(
                    barrier_second_derivative(sd, d_hat).to_bits(),
                    old_d2.to_bits(),
                    "b'' differs at sd = {sd:e}, d_hat = {d_hat:e}",
                );
                checked += 1;
            }
        }

        eprintln!(
            "  bit-identical at {checked} (sd, d_hat) points across 4 band widths\n                 of which {penetrating} penetrating (clamp path) and {near_edge} within \
             1e-9 of the band edge",
        );
        // Assert the COLLECTION, not just the absence of a failure: a sweep
        // that silently skipped everything would otherwise pass. Pinned to the
        // MEASURED counts rather than a guessed floor — the gap generation is
        // deterministic (the scatter is a seeded xorshift), so an inequality
        // here would just be a threshold I made up, and two earlier drafts of
        // this test failed on exactly that rather than on the arithmetic.
        assert_eq!(
            (checked, penetrating, near_edge),
            (596_162, 53_592, 4_000),
            "the sweep no longer covers what it did when these counts were \
             taken; re-read the three numbers before trusting the comparison",
        );
    }
}
