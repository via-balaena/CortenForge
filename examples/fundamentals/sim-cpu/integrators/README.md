# Integrator Examples

All 5 numerical integrators in CortenForge, demonstrated on pendulum systems.
Each per-integrator example includes an algorithm documentation block showing
the actual math the engine executes.

## The 5 integrators

| Integrator | Method | Stability | Cost | Factorization |
|------------|--------|-----------|------|---------------|
| **Euler** | Semi-implicit (symplectic) | Conditional | 1 eval | LDL (CRBA) |
| **RK4** | 4th-order Runge-Kutta | Good for smooth | 4 evals | None |
| **Implicit** | Full backward Euler + Coriolis | Unconditional | 1 eval + LU | LU (asymmetric) |
| **ImplicitFast** | Backward Euler, no Coriolis | Unconditional | 1 eval + Cholesky | Cholesky (symmetric) |
| **ImplicitSpringDamper** | Velocity-level, diagonal K/D | Unconditional (springs) | 1 eval + Cholesky | Cholesky |

## Examples

| Example | Type | What it shows |
|---------|------|---------------|
| [`comparison/`](comparison/) | Headless | All 5 integrators on the same pendulum, formatted table + 7 pass/fail checks |
| [`comparison-visual/`](comparison-visual/) | Bevy | 3 double pendulums (Euler, RK4, Implicit) side by side with energy HUD |
| [`euler/`](euler/) | Bevy | Semi-implicit Euler — visible energy drift at dt=0.005 |
| [`rk4/`](rk4/) | Bevy | 4th-order Runge-Kutta — near-zero energy drift |
| [`implicit/`](implicit/) | Bevy | Full implicit with Coriolis — unconditionally stable |
| [`implicit-fast/`](implicit-fast/) | Bevy | Simplified implicit, no Coriolis — Cholesky instead of LU |
| [`implicit-spring-damper/`](implicit-spring-damper/) | Bevy | Diagonal spring/damper implicit treatment |

## Run

```
cargo run -p example-integrator-comparison --release          # headless table
cargo run -p example-integrator-comparison-visual --release   # side-by-side
cargo run -p example-integrator-euler --release
cargo run -p example-integrator-rk4 --release
cargo run -p example-integrator-implicit --release
cargo run -p example-integrator-implicit-fast --release
cargo run -p example-integrator-implicit-spring-damper --release
```
