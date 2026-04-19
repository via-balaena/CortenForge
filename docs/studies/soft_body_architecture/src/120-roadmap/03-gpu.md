# Milestone — GPU acceleration

At the end of Phase E, `sim-soft` clears interactive rates on a consumer GPU. The canonical problem runs at ≥ 30 FPS in experience-mode (~5k tets) and ≥ 5 Hz in design-mode (~30k tets) on a single mid-tier consumer GPU — targets derived by scaling published NVIDIA Warp neo-Hookean benchmarks to comparable tet counts, per [Part 11 Ch 03](../110-crate/03-build-order.md#the-committed-order). They are *planning targets*, not measurements of `sim-soft`, and will be revised up or down as Phase E benchmarks land.

The deliverable is not only "the same thing, faster." Phase E changes the architecture of the forward solve. The inner-loop sparse matrix-vector product, preconditioned conjugate gradient, per-element neo-Hookean assembly, and IPC barrier evaluation all move to wgpu compute kernels ([Part 8 Ch 00](../80-gpu/00-wgpu-layout.md)). The autograd tape is extended to record GPU kernel calls and replay them in reverse ([Part 8 Ch 03](../80-gpu/03-gpu-autograd.md)), via the `sim-ml-chassis` extension committed in [Part 8 Ch 04](../80-gpu/04-chassis-extension.md).

## Deliverables

- **GPU forward solve** on the canonical problem, running against the Phase D CPU path as oracle. PR-level regression: a small scene (~500 tets) forward-solves on both paths and compares equilibrium configurations component-wise to 5 digits.
- **GPU autograd tape** recording and replaying kernel calls. Gradient of reward with respect to a material-field parameter agrees with the CPU path to 5 digits on the same small scene, and with central-difference finite differences to 5 digits on a shrunken problem where the FD cost is tolerable.
- **Preconditioned CG on GPU.** Per [Part 8 Ch 02](../80-gpu/02-sparse-solvers.md), the GPU linear solve wraps a Jacobi or block-Jacobi preconditioner around CG, with the preconditioner and warm-start vector persisted on the autograd tape so the IFT backward pass re-uses them. The tape pattern is [Part 5 Ch 00's factor-on-tape](../50-time-integration/00-backward-euler.md) extended to GPU.
- **Phase D CPU path retained as regression baseline.** Every PR touching GPU runs the small-scene regression on both paths. CPU is not retired; it is the oracle.

## Rate targets and their derivation

The 30 FPS and 5 Hz numbers back out to the solver throughput the canonical problem has to clear at each mode:

- **Experience mode, ~5k tets, 30 FPS.** One implicit step per frame, 2–3 Newton iterations per step, one preconditioned-CG solve per iteration with ~50 CG iterations. Roughly 100–150 sparse mat-vecs per frame on a mesh of this scale, plus per-element neo-Hookean assembly and barrier evaluation. This is within single-digit-millisecond range on a 4070-class GPU based on [Warp's published neo-Hookean benchmarks](../00-context/02-sota/01-warp.md).
- **Design mode, ~30k tets, 5 Hz.** Same per-step structure at 6× the tets. CG iteration count grows weakly in DOFs (driven by the stiffness matrix condition number, which scales modestly on mesh-derived sparse systems with a reasonable preconditioner), so total cost scales in a single-digit multiplier rather than quadratically, giving a per-step budget well inside the 5 Hz frame.

The targets shape the kernel design and the caching strategy in [Part 4 Ch 05](../40-contact/05-real-time.md). If Phase E benchmarks land outside the target band, the design-mode target is renegotiated *before* the SDF bridge is built on top of it, not after.

## What Phase E does not yet include

- **SDF authoring loop.** The GPU solver accepts meshes produced by a one-shot tetrahedralization of the SDF; live re-meshing is Phase G. Experience-mode previews at 30 FPS assume the mesh is fixed for the duration of the preview.
- **`sim-core` and `sim-thermostat` coupling.** Phase F. At Phase E close, the rigid probe is still a hardcoded SDF rather than a `sim-core` body.
- **Visual layer beyond diagnostic.** Phase I. A Phase E demo shows a flat-shaded tet mesh deforming at 30 FPS, not subsurface-scattered silicone.
- **Tet10 elements.** Phase H. Phase E's GPU port targets Tet4 only; the kernel design accommodates Tet10 in principle, but the correctness regression covers Tet4 until Phase H flips the switch.

## Exit criteria

Phase E closes when:

1. The GPU forward solve clears both rate targets on the canonical problem on at least one reference GPU (initial reference: Apple M-series or NVIDIA 4070-class).
2. The GPU-vs-CPU gradient-through-IFT regression passes at 5 digits on the small-scene suite for every PR in the last two weeks of Phase E.
3. The cross-vendor portability check from [Part 8 Ch 00](../80-gpu/00-wgpu-layout.md) passes on at least two distinct vendors (one Apple, one NVIDIA). A CUDA-native or Metal-native tuned path is explicitly deferred to post-Phase-I per Part 8 Ch 00.

Exit condition 3 is load-bearing: `sim-soft` is a cross-vendor platform, and a Phase E that runs only on one vendor would be a false close. If a vendor-specific bug blocks the portability check, it is resolved before Phase E lands — not after.
