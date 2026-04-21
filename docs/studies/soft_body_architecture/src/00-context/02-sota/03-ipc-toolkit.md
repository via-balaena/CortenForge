# IPC Toolkit — contact only

Repo: [`github.com/ipc-sim/ipc-toolkit`](https://github.com/ipc-sim/ipc-toolkit) · MIT · v1.5.0 (2026-02-06) · Primary paper: Li, Ferguson, Schneider, Langlois, Zorin, Jacobson, Panozzo 2020 "Incremental Potential Contact: Intersection- and Inversion-free Large Deformation Dynamics," ACM TOG 39(4) art. 49 (SIGGRAPH 2020) · Accessed: 2026-04-20

IPC Toolkit is the reference C++/Eigen implementation of Incremental Potential Contact — a smooth-barrier non-penetration contact formulation with continuous collision detection and smoothed Coulomb friction. It is **not a full simulation library**: the README explicitly states "This is not a full simulation library. As such, it does not include any physics or solvers." Python bindings via pybind11.

- **[Physically correct](../01-ceiling/00-definitions.md#physically-correct):** `partial`. Provides the IPC-grade-contact component of axis 1 but ships no hyperelastic material models, no implicit integration, no measured-data calibration. A hosting solver (e.g., [PolyFEM](00-polyfem.md)) integrates IPC Toolkit to cover the remaining three components of axis 1.
- **[Visually great](../01-ceiling/00-definitions.md#visually-great):** `—`. No rendering pipeline.
- **[Real-time](../01-ceiling/00-definitions.md#real-time):** `—`. Eigen-on-CPU; no GPU path in the library itself. [Part 4 Ch 05](../../40-contact/05-real-time.md)'s real-time IPC techniques are post-2020 follow-up work not integrated into the toolkit as a single-package real-time path. `sim-soft`'s [`contact/` module](../../110-crate/00-module-layout/03-contact.md) ports the algorithmic components to wgpu rather than using the C++ library directly.
- **[Differentiable](../01-ceiling/00-definitions.md#differentiable):** `—`. Barrier-function derivatives exist for Newton gradient and Hessian assembly but are not exposed as a general-purpose AD interface. Whether the derivatives are usable for IFT/adjoint sensitivities beyond solver-internal use — soft-deferred to Pass 2.

## What `sim-soft` inherits or learns

IPC Toolkit is the algorithmic reference for **axis 1's IPC-grade-contact component**. [Part 4 Ch 00](../../40-contact/00-why-ipc.md) and [Part 4 Ch 01](../../40-contact/01-ipc-internals.md) cite Li et al. 2020 directly; [Part 4 Ch 02](../../40-contact/02-friction.md) cites the same formulation for smoothed Coulomb friction. `sim-soft`'s relationship is an **algorithmic dependency** (the IPC math, re-implemented in Rust) and a **regression baseline** (per-scene contact-force and gap-distance fields against the library's outputs on shared fixtures) — not a linked dependency, because [Part 11 Ch 03](../../110-crate/03-build-order.md#the-committed-order)'s build-order targets Rust + wgpu rather than FFI into C++/Eigen.

## Citation status

- Repo + license + release cadence (v1.5.0 on 2026-02-06) confirmed from README.
- Primary paper: Li et al. 2020 ACM TOG 39(4) art. 49 — confirmed.
- AD exposure of barrier derivatives beyond Newton gradient/Hessian role — soft-deferred to Pass 2.
