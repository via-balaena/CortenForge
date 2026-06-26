//! Li 2020 smoothed-Coulomb friction — the position-space dissipative potential that
//! composes into the backward-Euler Newton minimization as one more energy term.
//!
//! Per active contact pair, with a lagged tangent frame `Tⁿ` (3×2, spanning the plane
//! ⊥ the normal `n̂`) and lagged normal-force magnitude `λⁿ ≥ 0`, the friction potential
//! over the tangential displacement `u_T = (Tⁿ)ᵀ(x_v − xᵗ)` since the step start `xᵗ` is
//!
//! ```text
//! D(x) = μ_c · λⁿ · f₀(‖u_T‖)
//! ```
//!
//! with the C¹ kernel `f₁` (force shape) and its C² antiderivative `f₀`:
//!
//! ```text
//! f₁(y) = −(y/w)² + 2(y/w)   (0 ≤ y ≤ w),  1   (y ≥ w)        w = h·ε_v
//! f₀(y) = w/3 − y³/(3w²) + y²/w  (y ≤ w),   y   (y ≥ w)        f₀' = f₁, f₀(w) = w
//! ```
//!
//! `w = h·ε_v` is the transition-zone width in displacement space (`ε_v` = velocity
//! threshold, m/s; `h` = timestep). The friction FORCE on the soft vertex is `−∇_x D`;
//! this module returns `∇_x D` and `∇²_x D` (the value added into the residual / tangent).
//! `f₁ ≤ 1` makes the force respect the Coulomb cone `‖force‖ ≤ μ_c·λⁿ`, and `f₀`'s
//! quadratic bowl at the origin (`f₀(‖u‖) ≈ w/3 + ‖u‖²/w`) makes `D` C² through `u_T = 0`,
//! so the Newton Hessian stays continuous (and PSD) across the stick→slip transition.
//! `λⁿ` and `Tⁿ` are LAGGED — recomputed from the current iterate and held constant for
//! the assembly (only `u_T` is differentiated) — the standard IPC friction lagging.
//!
//! See `docs/keystone/friction_recon.md` and
//! `docs/studies/soft_body_architecture/src/40-contact/02-friction/01-smoothed.md`.

use crate::Vec3;
use nalgebra::{Matrix2, Matrix3, Matrix3x2, Vector2};

/// The C¹ smoothing kernel `f₁(y) ∈ [0, 1]` (`y, w` in displacement units).
#[must_use]
fn f1(y: f64, w: f64) -> f64 {
    if y >= w {
        1.0
    } else {
        let xi = y / w;
        xi * (2.0 - xi) // −ξ² + 2ξ
    }
}

/// `f₁'(y)` — used by the Hessian. `(2/w)(1 − y/w)` inside the band, `0` outside.
#[must_use]
fn f1_prime(y: f64, w: f64) -> f64 {
    if y >= w { 0.0 } else { 2.0 / w * (1.0 - y / w) }
}

/// The C² antiderivative `f₀` (`f₀' = f₁`), normalized so `f₀(w) = w`.
#[must_use]
pub(crate) fn f0(y: f64, w: f64) -> f64 {
    if y >= w {
        y
    } else {
        w / 3.0 - y * y * y / (3.0 * w * w) + y * y / w
    }
}

/// A lagged tangent basis `Tⁿ` (3×2) spanning the plane ⊥ `n̂` (sign of `n̂` is
/// irrelevant — the tangent plane is the same for `±n̂`).
#[must_use]
pub(crate) fn tangent_basis(n: Vec3) -> Matrix3x2<f64> {
    let n = n.normalize();
    // Pick a seed axis least aligned with `n`, Gram-Schmidt it off, complete the frame.
    let seed = if n.x.abs() < 0.9 {
        Vec3::x()
    } else {
        Vec3::y()
    };
    let t0 = (seed - n * seed.dot(&n)).normalize();
    let t1 = n.cross(&t0);
    Matrix3x2::from_columns(&[t0, t1])
}

/// The friction potential value `D = μ·λⁿ·f₀(‖u_T‖)` for one pair (for tests / energy
/// readouts). `x_start` is the step-start position `xᵗ` of the vertex.
#[must_use]
pub(crate) fn potential(
    x_v: Vec3,
    x_start_v: Vec3,
    normal: Vec3,
    lambda_n: f64,
    mu: f64,
    w: f64,
) -> f64 {
    let t = tangent_basis(normal);
    let u: Vector2<f64> = t.transpose() * (x_v - x_start_v);
    mu * lambda_n * f0(u.norm(), w)
}

/// The friction gradient `∇_x D` (the force added into the residual is this; the force ON
/// the soft vertex is its negation) AND Hessian `∇²_x D` (3×3, PSD) for one pair, with the
/// lagged `(normal, λⁿ)` held constant (only `u_T` differentiated). `w = h·ε_v`.
///
/// Gradient: `μ·λⁿ·f₁(r)·T·û`  (`r = ‖u_T‖`, `û = u_T/r`).
/// Hessian:  `μ·λⁿ·T·[ (f₁(r)/r)(I₂ − û⊗û) + f₁'(r)·û⊗û ]·Tᵀ`, isotropic `μ·λⁿ·(2/w)·T·Tᵀ`
/// at `r = 0` (the smooth bowl). Both PSD (`f₁ ≥ 0`, `f₁/r ≥ 0`, `f₁' ≥ 0`).
// many_single_char_names: `t`/`u`/`r`/`g`/`h` are the standard symbols of the friction
// math (tangent basis, displacement, its norm, the 2-D gradient/Hessian) — renaming hurts
// the correspondence to the formulas above and the spec.
#[allow(clippy::many_single_char_names)]
#[must_use]
pub(crate) fn grad_hess(
    x_v: Vec3,
    x_start_v: Vec3,
    normal: Vec3,
    lambda_n: f64,
    mu: f64,
    w: f64,
) -> (Vec3, Matrix3<f64>) {
    let scale = mu * lambda_n;
    if scale == 0.0 {
        return (Vec3::zeros(), Matrix3::zeros());
    }
    let t = tangent_basis(normal);
    let u: Vector2<f64> = t.transpose() * (x_v - x_start_v);
    let r = u.norm();

    // The tangent-space (2×2) Hessian of f₀(‖u‖); handle r → 0 with the bowl limit.
    let (grad2, h2): (Vector2<f64>, Matrix2<f64>) = if r < 1e-300 {
        // f₁(0) = 0 ⇒ no force; ∇²f₀(0) = (2/w)·I₂ (the isotropic bowl).
        (Vector2::zeros(), Matrix2::identity() * (2.0 / w))
    } else {
        let uhat = u / r;
        let uu = uhat * uhat.transpose(); // û⊗û
        let g = f1(r, w) * uhat;
        let h = f1(r, w) / r * (Matrix2::identity() - uu) + f1_prime(r, w) * uu;
        (g, h)
    };
    let grad = scale * (t * grad2);
    let hess = scale * (t * h2 * t.transpose());
    (grad, hess)
}

/// The friction force's **normal-rotation Jacobian** `DN = ∂(∇D)/∂n̂` (3×3) — how the
/// friction force turns as the contact NORMAL turns, with the lagged `(λⁿ, slip)` held.
/// Zero contribution for a flat collider (the normal is constant), the missing curved-contact
/// term for a finite primitive whose normal rotates as the contact point slides over it
/// (`∂n̂/∂x = H`) or as the primitive translates (`∂n̂/∂pose = −H·dir`). The curved-normal
/// dual of the in-plane slip Hessian [`grad_hess`] returns: where that differentiates the
/// tangential displacement `u_T` (normal frozen), this differentiates the tangent frame.
///
/// `∇D = μ·λⁿ·g(u_t)` with `g(u_t) = f₁(r)·u_t/r`, `r = ‖u_t‖`, and the basis-free tangential
/// displacement `u_t = (I − n̂n̂ᵀ)·Δ`, `Δ = x_v − x_start` (`= T·u_T`, identical force). Only
/// `u_t` carries the `n̂`-dependence, so the chain rule gives
///
/// ```text
/// DN = μ·λⁿ · J_g · (∂u_t/∂n̂),
///   J_g     = (f₁/r)(I − ûûᵀ) + f₁'·ûûᵀ          (the full 3-D force Jacobian, û = u_t/r)
///   ∂u_t/∂n̂ = −[ (n̂·Δ)·I + n̂·Δᵀ ]                (from u_t = Δ − n̂(n̂·Δ))
/// ```
///
/// At `r = 0` (the smooth bowl) `J_g → (2/w)·I` — the same isotropic limit [`grad_hess`] uses.
/// A caller chains this with `∂n̂/∂x = H = ∇²sd` (state) or `∂n̂/∂pose = −H·dir` (pose): the
/// reaction Jacobian's `DN·H` term and the friction pose-residual's `DN·(−H·dir)` term. `H = 0`
/// for a plane makes every such chain identically zero (flat-collider byte-identity).
// `t`/`u`/`r`/`g` match `grad_hess`'s symbols (this is its curved-normal companion).
#[allow(clippy::many_single_char_names)]
#[must_use]
pub(crate) fn normal_rotation_term(
    x_v: Vec3,
    x_start_v: Vec3,
    normal: Vec3,
    lambda_n: f64,
    mu: f64,
    w: f64,
) -> Matrix3<f64> {
    let scale = mu * lambda_n;
    if scale == 0.0 {
        return Matrix3::zeros();
    }
    let n = normal.normalize();
    let delta = x_v - x_start_v;
    // u_t = (I − n̂n̂ᵀ)·Δ (the tangential slip, basis-free); r = ‖u_t‖.
    let u_t = delta - n * n.dot(&delta);
    let r = u_t.norm();
    // J_g = ∂g/∂u_t — the full 3-D friction-force Jacobian (the bowl limit (2/w)·I at r → 0).
    let j_g: Matrix3<f64> = if r < 1e-300 {
        Matrix3::identity() * (2.0 / w)
    } else {
        let uhat = u_t / r;
        let uu = uhat * uhat.transpose();
        f1(r, w) / r * (Matrix3::identity() - uu) + f1_prime(r, w) * uu
    };
    // ∂u_t/∂n̂ = −[(n̂·Δ)I + n̂Δᵀ].
    let du_dn = -(Matrix3::identity() * n.dot(&delta) + n * delta.transpose());
    scale * (j_g * du_dn)
}

#[cfg(test)]
mod tests {
    use super::*;

    const MU: f64 = 0.5;
    const LAMBDA: f64 = 800.0;
    const W: f64 = 1.0e-5; // h·ε_v

    fn fd_grad(x: Vec3, xs: Vec3, n: Vec3) -> Vec3 {
        let eps = 1e-9;
        let mut g = Vec3::zeros();
        for i in 0..3 {
            let (mut xp, mut xm) = (x, x);
            xp[i] += eps;
            xm[i] -= eps;
            g[i] = (potential(xp, xs, n, LAMBDA, MU, W) - potential(xm, xs, n, LAMBDA, MU, W))
                / (2.0 * eps);
        }
        g
    }

    fn fd_hess(x: Vec3, xs: Vec3, n: Vec3) -> Matrix3<f64> {
        let eps = 1e-7;
        let mut h = Matrix3::zeros();
        for i in 0..3 {
            let (mut xp, mut xm) = (x, x);
            xp[i] += eps;
            xm[i] -= eps;
            let gp = grad_hess(xp, xs, n, LAMBDA, MU, W).0;
            let gm = grad_hess(xm, xs, n, LAMBDA, MU, W).0;
            let col = (gp - gm) / (2.0 * eps);
            for r in 0..3 {
                h[(r, i)] = col[r];
            }
        }
        h
    }

    #[test]
    fn grad_and_hess_match_fd() {
        let n = Vec3::z();
        let xs = Vec3::new(0.05, -0.02, 0.1);
        // slip > 0, away from the seam r = W (where the Hessian is continuous but kinked —
        // f₀ is C² but ∇²D is non-differentiable there, so an FD straddling it mismatches):
        // band interior, slip interior, deep slip.
        for slip in [0.3 * W, 2.0 * W, 50.0 * W] {
            let x = xs + Vec3::new(slip * 0.6, slip * 0.8, 0.0);
            let (g, h) = grad_hess(x, xs, n, LAMBDA, MU, W);
            let rel_g = (g - fd_grad(x, xs, n)).norm() / fd_grad(x, xs, n).norm().max(1e-12);
            assert!(rel_g < 1e-5, "∇D vs FD at slip {slip:.1e}: rel {rel_g:.2e}");
            let rel_h = (h - fd_hess(x, xs, n)).norm() / fd_hess(x, xs, n).norm().max(1e-12);
            assert!(
                rel_h < 1e-4,
                "∇²D vs FD at slip {slip:.1e}: rel {rel_h:.2e}"
            );
            assert!(
                (h - h.transpose()).norm() < 1e-9 * h.norm().max(1.0),
                "Hessian symmetric"
            );
        }
        // slip = 0: zero force + the isotropic bowl limit ∇²D = μλ·(2/w)·T·Tᵀ (the FD of
        // a finite step would land in the band where the true Hessian differs by O(eps/w),
        // so check the limit analytically, not via FD).
        let (g0, h0) = grad_hess(xs, xs, n, LAMBDA, MU, W);
        assert!(g0.norm() < 1e-12, "zero force at zero slip");
        let t = tangent_basis(n);
        let bowl = MU * LAMBDA * (2.0 / W) * (t * t.transpose());
        assert!(
            (h0 - bowl).norm() / bowl.norm() < 1e-12,
            "bowl-limit Hessian at slip 0"
        );
    }

    /// Moving-collider drift, hand-checked: a soft vertex at REST (`x_v = xᵗ`) whose
    /// friction reference is shifted by the rigid surface's within-step tangential drift
    /// `Δ_surf` (i.e. effective start `xᵗ_eff = xᵗ + Δ_surf`) feels a friction force in
    /// the `+Δ_surf` direction (the collider DRAGS it along), magnitude `μλ` once the
    /// drift exceeds the stick band `w`. This is the sign + frame the solver relies on
    /// for two-way grip; the solver applies the shift inside `friction_blocks`.
    #[test]
    fn drift_drags_resting_vertex_in_drift_direction() {
        let n = Vec3::z();
        let xt = Vec3::new(0.05, -0.02, 0.1); // step-start = current (vertex at rest)
        // Deep-slip drift (‖Δ‖ = 5w ≫ w) ⇒ saturated f₁ = 1 ⇒ force = μλ exactly.
        for dir in [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.6, -0.8, 0.0),
            Vec3::new(-0.3, 0.4, 0.7), // the normal (z) part must be projected out
        ] {
            let dir_t = Vec3::new(dir.x, dir.y, 0.0); // tangential part of the drift dir
            let drift = 5.0 * W * dir / dir.norm();
            let x_start_eff = xt + drift;
            // force on soft = −∇D (friction_blocks negates grad for the residual scatter).
            let force_on_soft = -grad_hess(xt, x_start_eff, n, LAMBDA, MU, W).0;
            // Direction: along +Δ_surf (tangential), i.e. the drag pulls the vertex the
            // way the collider swept.
            let expected = MU * LAMBDA * dir_t / dir_t.norm();
            assert!(
                (force_on_soft - expected).norm() / expected.norm() < 1e-12,
                "drift {drift:?}: force {force_on_soft:?} vs expected μλ·Δ̂ {expected:?}"
            );
        }
        // Zero drift + resting vertex ⇒ no slip ⇒ no force (PR1 byte-identical floor).
        let f0 = -grad_hess(xt, xt, n, LAMBDA, MU, W).0;
        assert!(
            f0.norm() < 1e-12,
            "zero drift at rest ⇒ zero friction, got {f0:?}"
        );
    }

    /// `normal_rotation_term` (`DN = ∂∇D/∂n̂`) matches a finite difference of the friction
    /// force `grad_hess.0` w.r.t. the (normalized) contact normal. `grad_hess` normalizes its
    /// `normal`, so FD-ing it measures `DN·(I − n̂n̂ᵀ)` (the perturbation projects ⊥ n̂ — exactly
    /// the curved-collider regime, where `∂n̂/∂x = H` is itself a `(I − n̂n̂ᵀ)`-image), so the
    /// analytic is compared after the same projection. Slip > 0 (away from the bowl r = 0 and
    /// the f₁ seam r = w, where the term is kinked).
    #[test]
    fn normal_rotation_matches_fd() {
        let n = Vec3::new(0.2, -0.3, 1.0).normalize();
        let xs = Vec3::new(0.05, -0.02, 0.1);
        let proj = Matrix3::identity() - n * n.transpose();
        let eps = 1e-7;
        for slip in [0.3 * W, 2.0 * W, 50.0 * W] {
            // A slip direction with a real tangential component (project off n̂).
            let dir = Vec3::new(0.6, 0.8, 0.4);
            let dir_t = dir - n * n.dot(&dir);
            let x = xs + slip * dir_t.normalize();
            let dn = normal_rotation_term(x, xs, n, LAMBDA, MU, W);
            let expected = dn * proj;
            // Central FD of grad_hess.0 w.r.t. the normal vector.
            let mut fd = Matrix3::zeros();
            for j in 0..3 {
                let (mut np, mut nm) = (n, n);
                np[j] += eps;
                nm[j] -= eps;
                let col = (grad_hess(x, xs, np, LAMBDA, MU, W).0
                    - grad_hess(x, xs, nm, LAMBDA, MU, W).0)
                    / (2.0 * eps);
                for r in 0..3 {
                    fd[(r, j)] = col[r];
                }
            }
            let rel = (expected - fd).norm() / fd.norm().max(1e-12);
            assert!(
                rel < 1e-5,
                "DN vs FD at slip {slip:.1e}: rel {rel:.2e}\nexpected {expected}\nfd {fd}"
            );
        }
    }

    #[test]
    fn respects_coulomb_cone() {
        let n = Vec3::z();
        let xs = Vec3::zeros();
        // Deep slip ⇒ force magnitude = μ·λ exactly; never exceeds it anywhere.
        let f = grad_hess(xs + Vec3::new(1.0, 0.0, 0.0), xs, n, LAMBDA, MU, W)
            .0
            .norm();
        assert!(
            (f - MU * LAMBDA).abs() / (MU * LAMBDA) < 1e-12,
            "deep slip = μλ"
        );
        for slip in [0.1 * W, 0.5 * W, W, 2.0 * W, 10.0 * W] {
            let g = grad_hess(xs + Vec3::new(slip, 0.0, 0.0), xs, n, LAMBDA, MU, W).0;
            assert!(g.norm() <= MU * LAMBDA + 1e-9, "force within the cone");
        }
    }
}
