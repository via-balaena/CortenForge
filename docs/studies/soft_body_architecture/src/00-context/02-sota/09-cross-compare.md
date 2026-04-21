# Cross-compare table

The full side-by-side of the nine SOTA tools against the [four ceiling axes from §00](../01-ceiling/00-definitions.md). Scoring per each sub-leaf's §00-rubric judgment, which may differ from the parent [SOTA survey table](../02-sota.md) — the sub-leaves are authoritative; the parent will be reconciled at Ch 02-close. Accessed: 2026-04-20.

## The synthesis row

| Tool | Physical | Visual | Real-time | Diff |
|---|:---:|:---:|:---:|:---:|
| [PolyFEM](00-polyfem.md) | ✓ | — | — | — |
| [NVIDIA Warp](01-warp.md) | partial | — | ✓ | ✓ |
| [Genesis](02-genesis.md) | partial | partial | partial | partial |
| [IPC Toolkit](03-ipc-toolkit.md) | partial | — | — | — |
| [DiffTaichi](04-difftaichi.md) | partial | — | ✓ | ✓ |
| [NVIDIA Flex](05-nvidia-flex.md) | — | ✓ | ✓ | — |
| [Houdini FEM Solver](06-houdini-jelly.md) | partial | ✓ | — | — |
| [FEBio](07-febio.md) | partial | — | — | — |
| [MuJoCo flex](08-mujoco-flex.md) | partial | — | ✓ | — |

**No tool has `✓` in all four columns.** That is the gap [§02](../01-ceiling/02-the-gap.md) names.

## What the columns say

**Physical:** Only PolyFEM reaches full axis-1 `✓` (hyperelastic + IPC + implicit + measured-data). Seven tools score `partial`: six are missing the IPC component (Warp, Genesis, DiffTaichi, Houdini FEM, FEBio, MuJoCo flex), and IPC Toolkit is the scope-limited contact-only library that *provides* IPC but carries no FEM solver, hyperelastic material models, or implicit integration. Only Flex scores `—` on Physical (PBD, not hyperelastic).

**Visual:** Only Flex and Houdini FEM Solver reach `✓` — the two tools that commit to physics-driven pixels as a first-class pathway. Genesis is `partial` (ships a photorealistic renderer but physics-render fusion not verified in the 2026-04-20 fetch). Everyone else is `—` (no integrated rendering pathway at the axis-2 ceiling).

**Real-time:** Four tools reach `✓` — Warp, DiffTaichi, Flex, and MuJoCo flex. Genesis is `partial` (interactive-rate claimed; specific benchmarks soft-deferred). Four offline-correctness FEM stacks score `—` (PolyFEM, IPC Toolkit, Houdini FEM Solver, FEBio).

**Differentiable:** Only Warp and DiffTaichi reach full `✓`. Genesis is `partial` (subset of solvers differentiable; rigid and articulated paths roadmapped). Everyone else scores `—`, including the offline-correctness FEM stacks and the position-based games-branch stacks.

## The empty pairwise intersections

The cross-product of axes tells the gap story:

- **Physical `✓` AND Visual `✓`:** zero tools.
- **Physical `✓` AND Real-time `✓`:** zero tools. (Warp and DiffTaichi are `partial` on Physical; the offline FEM stacks are `—` on Real-time.)
- **Physical `✓` AND Differentiable `✓`:** zero tools. (Same reason.)
- **Visual `✓` AND Real-time `✓`:** only Flex — but Flex is `—` on Physical and `—` on Differentiable.
- **Visual `✓` AND Differentiable `✓`:** zero tools.
- **Real-time `✓` AND Differentiable `✓`:** Warp and DiffTaichi — but both are `partial` on Physical and `—` on Visual.

**Three-way intersections:** zero tools hit three `✓`s simultaneously. The closest is Warp (`partial` / `—` / `✓` / `✓`) which reaches two full `✓`s but falls short on Physical and Visual.

**Four-way intersection:** zero tools. This is the integration surface [`sim-soft`](../../110-crate/00-module-layout.md) ships — the one solver that carries `✓` on all four axes simultaneously under [§00's rubric](../01-ceiling/00-definitions.md).

## Caveats on this snapshot

- Scoring is per each sub-leaf's 2026-04-20 verification pass. Citation soft-defers (contact-formulation specifics, benchmark citations, FEBio and PolyFEM material-catalog completeness, MJX-flex differentiability) are tracked per-leaf and will be reconciled in Pass 2.
- The parent [SOTA survey table](../02-sota.md) currently has different `✓`/partial/— entries in several cells — notably Physical `✓` for Warp, Genesis, IPC Toolkit, Houdini FEM, and FEBio; Diff `partial` for PolyFEM; Real-time `✓` for Genesis. Sub-leaf scoring is authoritative per the [§00 rubric](../01-ceiling/00-definitions.md); the parent table will be updated at Ch 02-close to match.
- Genesis scores `partial` across all four axes not because it fails each, but because each was unverified in the 2026-04-20 fetch. Pass 2 may upgrade some of Genesis's axes to `✓` once specific capabilities are confirmed via a deeper README + examples walk.
- The skin + cloth + hair unified-solver dimension from [§02 the-gap](../01-ceiling/02-the-gap.md)'s missing-combination list is orthogonal to the four-axis scoring and is not represented in the table above; no tool unifies those primitives in one integrated solver, which is a separate gap from the four-axis coverage.
