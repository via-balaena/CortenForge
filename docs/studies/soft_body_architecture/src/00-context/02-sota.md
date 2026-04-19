# SOTA survey

The soft-body simulation literature contains dozens of solvers; this chapter focuses on the nine that are closest to the ceiling defined in [Ch 01](01-ceiling.md) or that occupy a position the book's design has to react to. Each sub-chapter covers one solver, naming what it does well, what it does not, which of the ceiling's four axes it satisfies, and what `sim-soft` inherits from it — as a dependency, an algorithmic component, a regression baseline, or a negative example the design has to beat.

The rightmost column below is the bill of indictment. Every capability `sim-soft` needs exists in at least one of these solvers; no solver has all of them; the integration is the gap.

| Solver | Strength | Key weakness | Physical | Visual | Real-time | Diff |
|---|---|---|:---:|:---:|:---:|:---:|
| [PolyFEM](02-sota/00-polyfem.md) | IPC + hyperelastic FEM, research-grade correct | Offline; CPU; C++ | ✓ | — | — | partial |
| [NVIDIA Warp](02-sota/01-warp.md) | GPU neo-Hookean, full autograd | No IPC; Python-first; NVIDIA-only | ✓ | — | ✓ | ✓ |
| [Genesis](02-sota/02-genesis.md) | Differentiable, GPU, multi-physics | Python; pre-release; research-grade | ✓ | partial | ✓ | ✓ |
| [IPC Toolkit](02-sota/03-ipc-toolkit.md) | Reference IPC implementation | Contact only; not a full solver; CPU | ✓ | — | — | — |
| [DiffTaichi](02-sota/04-difftaichi.md) | Differentiable physics in a DSL | Simpler constitutive models; DSL lock-in | partial | — | ✓ | ✓ |
| [NVIDIA Flex](02-sota/05-nvidia-flex.md) | Position-based, GPU, production-mature | PBD — not hyperelastic, not differentiable | — | ✓ | ✓ | — |
| [Houdini Jelly](02-sota/06-houdini-jelly.md) | Production-quality visual, multi-material | Offline; proprietary; not a library | ✓ | ✓ | — | — |
| [FEBio](02-sota/07-febio.md) | Biomechanics hyperelastic + viscoelastic | Offline CPU; biomech scope only | ✓ | — | — | — |
| [MuJoCo flex](02-sota/08-mujoco-flex.md) | Already in our engine; Track 1B baseline | Beta; penalty contact; not differentiable | partial | — | ✓ | — |
| [Cross-compare table](02-sota/09-cross-compare.md) | Full side-by-side on all four axes | — | | | | |

Two claims Ch 02 rests on:

1. **No existing solver covers all four axes simultaneously.** Warp and Genesis come closest but lack IPC-quality contact; PolyFEM and FEBio have the physics but run offline on CPU; Flex and Jelly have one side or the other but not both. The empty cells in the rightmost columns above are the gap [Ch 01 sub-chapter 02](01-ceiling/02-the-gap.md) names.
2. **The gap is not research — it is integration.** Every capability `sim-soft` needs has been demonstrated in at least one solver above. The book's contribution is the integration, in Rust, on wgpu, with [`sim-core`](../110-crate/02-coupling/00-mjcf.md), [`sim-thermostat`](../110-crate/02-coupling/01-thermostat.md), and [`cf-design`](../110-crate/02-coupling/04-cf-design.md) as first-class siblings rather than afterthoughts or adapters.
