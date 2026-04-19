# Bind group design

A wgpu `BindGroup` is the API-level handle that binds a set of buffers and textures to a specific `@group(n) @binding(m)` layout declared by a compute shader. Every dispatch has to set up one bind group per declared group before it runs. At `sim-soft`'s ≈80 (AMG-path) to ≈700 (Jacobi-path) kernel invocations per Newton iteration ([§02 async](02-async.md) on the preconditioner-dependent range), bind-group construction is a hot path — and the wrong abstraction here turns a clean pipeline into a command-encoder swamp that dominates the CPU-side driver cost. The [Ch 00 parent's table](../00-wgpu-layout.md) names the split into *scene-static* and *per-step* tiers; this leaf specifies both.

## The two-tier layout

**Group 0 — scene-static.** Bound once per frame (or per episode, depending on workload). Holds buffers whose contents are fixed across the Newton iteration:

- `tet_connectivity: array<vec4<u32>>` — the per-tet vertex indices. Rebuilt only on [Part 7 Ch 04 re-mesh](../../70-sdf-pipeline/04-live-remesh.md); constant within a timestep.
- `reference_geometry: array<mat4x3<f32>>` — the per-tet $B_e$ matrix (reference-space shape-function gradient, 3-row × 4-column; in WGSL's column-major-first naming this is `mat4x3<f32>`). Fixed across the episode unless a re-mesh fires.
- `material_params: array<f32>` — Young's modulus, Poisson's ratio, density per-tet (one value each, interleaved). Fixed across the episode.
- `bsr_row_ptr: array<u32>` + `bsr_col_idx: array<u32>` — the sparse-matrix structural pattern. Rebuilt on re-mesh; constant within a timestep.

**Group 1 — per-step.** Bound per compute pass within the Newton iteration. Holds buffers whose contents change each iteration:

- `positions: array<vec3<f32>>` — the current Newton iterate $x^{(k)}$.
- `velocities: array<vec3<f32>>` — the current velocity (for line-search guards; updated per accepted Newton step).
- `gradient: array<vec3<f32>>` — the assembled global gradient $g^{(k)}$. Scatter target for per-element gradient writes.
- `hessian_values: array<f32>` — the assembled global Hessian values in BSR block layout. Scatter target for per-element stiffness writes. Same storage buffer, but the WGSL type changes with kernel access: assembly kernels view it as `array<atomic<u32>>` (scatter via u32-aliased compare-and-swap, see below); SpMV kernels view it as `array<f32>` (read-only). Rust-side code owns one `wgpu::Buffer`; the WGSL-side declaration chooses the type per bind-group layout.
- `active_contact_pairs: array<ContactPair>` — the IPC active-pair list for this iteration. Rebuilt by broadphase each Newton step.

The split is by mutation frequency, not by logical role. Connectivity and reference geometry could arguably live with positions (they are all mesh-attached), but mutation frequency is what drives bind-group invalidation, and invalidating a bind group forces re-construction which costs ≈2 μs of CPU driver time per group. At 80 kernels × 2 groups × 10 Newton iters (the AMG-path low end), re-constructing *both* groups per kernel would add ≈3 ms of CPU overhead per timestep — more than the GPU compute in some per-DOF kernels. On the Jacobi-path end (≈700 kernels/iter), the same naïve scheme scales to ≈28 ms and dominates the step. The two-tier split amortizes the scene-static group across the full Newton iteration and keeps the per-step group small enough that its reconstruction is negligible.

## Per-workload-shape bind-group layouts

Each of the four workload shapes from [§00 kernel-types](00-kernel-types.md) has its own bind-group layout specification. The layouts are declared once at chassis init and re-used across all kernels of that shape:

**Per-element layout** (Group 1 variant for the assembly dispatch):

```wgsl
@group(0) @binding(0) var<storage, read>  tet_connectivity: array<vec4<u32>>;
@group(0) @binding(1) var<storage, read>  reference_geometry: array<mat4x3<f32>>;
@group(0) @binding(2) var<storage, read>  material_params: array<f32>;
@group(1) @binding(0) var<storage, read>  positions: array<vec3<f32>>;
@group(1) @binding(1) var<storage, read_write> hessian_values: array<atomic<u32>>;
@group(1) @binding(2) var<storage, read_write> gradient: array<atomic<u32>>;
```

The `atomic<u32>` declaration on the scatter targets is the WGSL spelling for "this buffer supports atomic operations." Atomic `f32` is not natively supported in WGSL; the per-element assembly kernel packs `f32` values into `u32` via bit-reinterpretation and performs compare-and-swap loops for the float atomics. This is the pattern [Ch 03 §01 playback](../03-gpu-autograd/01-playback.md) also uses for grad accumulation; the cost is ≈3× a native atomic add on mid-tier hardware, and the constraint is WGSL's, not ours.

**Per-contact-pair layout** (swaps the per-element buffers for the active-pair list):

```wgsl
@group(0) @binding(0) var<storage, read> tet_connectivity: array<vec4<u32>>;  // for primitive lookup
@group(0) @binding(1) var<storage, read> barrier_params: array<f32>;          // per-primitive d_hat
@group(1) @binding(0) var<storage, read> positions: array<vec3<f32>>;
@group(1) @binding(1) var<storage, read> active_pairs: array<ContactPair>;
@group(1) @binding(2) var<storage, read_write> gradient: array<atomic<u32>>;
@group(1) @binding(3) var<storage, read_write> hessian_contact: array<atomic<u32>>;
```

The contact Hessian scatters into a *separate* BSR buffer from the elastic Hessian, which is summed at dispatch time via a third kernel. The reason is that contact-pair Hessian contributions have a different sparsity pattern than elastic stiffness (two-vertex fan-out rather than four-vertex fan-out), and fusing them into one buffer would require rebuilding the sparsity pattern per iteration — which the adaptive-contact-list machinery already makes expensive enough.

**Per-DOF layout** (CG inner loop):

```wgsl
@group(0) @binding(0) var<storage, read> hessian_row_ptr: array<u32>;      // BSR structural
@group(0) @binding(1) var<storage, read> hessian_col_idx: array<u32>;      // BSR structural
@group(1) @binding(0) var<storage, read> hessian_values: array<f32>;       // BSR numerical
@group(1) @binding(1) var<storage, read> input_vector: array<vec3<f32>>;
@group(1) @binding(2) var<storage, read_write> output_vector: array<vec3<f32>>;
```

Note that `hessian_values` is per-step (Group 1) but `hessian_row_ptr` and `hessian_col_idx` are scene-static (Group 0). This is the BSR structural/numerical split: the structure depends only on the mesh, the values depend on the iterate. The split lets SpMV's Group 0 stay bound across every CG iteration, with only Group 1 (values + input + output) re-bound per iteration.

**Per-tet-state layout** (state transfer and readout): the specific bindings vary by kernel (state-transfer reads source-mesh positions + writes target-mesh positions, while stress-readout reads positions + writes per-tet stress tensors), so this shape does not pin a single canonical layout — each kernel declares its own Group 1. Group 0 remains the scene-static mesh data.

## Layout declarations are chassis-level, not per-kernel

All four layouts are declared at chassis init as `wgpu::BindGroupLayout` objects and stored in the GPU context. Individual kernels reference them by handle; a kernel that implements "per-element assembly" does not re-declare the layout, it looks up the chassis's `BindGroupLayout::PerElementAssembly` handle. This is a deliberate enforcement of the "four shapes" taxonomy from [§00](00-kernel-types.md): a new kernel has to pick an existing shape, and if none fits, the shape taxonomy has to be extended at the chassis level, not slipped in via ad-hoc layout.

The layout registry is the `sim-ml-chassis::gpu::BindGroupLayoutRegistry` named in [Ch 04 §01](../04-chassis-extension/01-gpu-backend.md). It is shared across all `sim-soft` GPU kernels and any future chassis consumers ([sim-rl](../../110-crate/03-build-order.md), [sim-opt](../../110-crate/03-build-order.md)) that ship their own GPU kernels. The taxonomy is a contract, not a convention.

## Alignment and packing constraints

WGSL imposes alignment rules that differ from Rust's default struct layout, and the gap between the two has caused more GPU bugs in practice than any algorithmic issue. Three constraints worth naming:

- **`vec3<f32>` is 16-byte-aligned in storage buffers.** Three `f32`s stored as a `vec3<f32>` occupy 16 bytes on disk, not 12. `sim-soft`'s GPU positions, velocities, and gradient buffers are allocated with 16-byte stride per vertex, not 12-byte. CPU-side layout code mirrors this explicitly — a padding `f32` per vertex — to keep memcpy-based uploads valid.
- **`mat4x3<f32>` is column-major and padded.** $B_e$ is a 3-row × 4-column matrix (maps 4 tet-vertex contributions into a 3D gradient); in WGSL naming this is `mat4x3<f32>` — 4 columns, each a `vec3<f32>` padded to 16 bytes. Total size is 64 bytes per tet (4 columns × 16 bytes each). The reference-geometry buffer layout accounts for this; the CPU-side `B_e` matrix is stored column-major with the pad, not packed.
- **Struct members cannot straddle 16-byte boundaries unless the struct itself is 16-byte-padded.** The `ContactPair` struct declared in `active_pairs` follows WGSL's struct layout rules — fields are laid out sequentially with padding inserted before any field that would otherwise straddle a boundary. `sim-soft`'s CPU-side `ContactPair` uses `#[repr(C)]` plus explicit `_pad: u32` fields where WGSL's implicit padding would have gone.

These are driver-level constraints, not `sim-soft` choices. Violating them produces silent data corruption that looks like correctness bugs in the solver. Phase E's bring-up includes a layout-verification pass that cross-checks CPU and GPU struct sizes at dispatch time.

## What this sub-leaf commits the book to

- **Two bind-group tiers: scene-static (Group 0) and per-step (Group 1).** The split is by mutation frequency. Per-frame or per-episode data binds once per frame; per-Newton-iteration data binds per dispatch.
- **Four canonical layout types, one per workload shape.** Per-element, per-contact-pair, per-DOF, per-tet-state. Declared at chassis init; kernels reference by handle; the taxonomy is a contract.
- **Atomic float scatter uses `atomic<u32>` + bit-reinterpret.** WGSL does not provide native `atomic<f32>` on all backends; `sim-soft`'s assembly scatter kernels use compare-and-swap loops on `u32`-aliased `f32` buffers. ≈3× slower than native atomic add but portable.
- **WGSL alignment rules are enforced at the CPU-Rust side via explicit padding.** `vec3<f32>` is 16-aligned, matrices are column-major-padded, structs are 16-byte-padded where needed. The `sim-ml-chassis::gpu` tensor layout types mirror this exactly; any mismatch is a Phase E bring-up bug, not a design choice.
