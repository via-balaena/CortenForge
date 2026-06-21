# Perf / scale baseline — CPU vs GPU-batched physics

Honest, measured throughput of the CortenForge physics step across a batch-size
sweep, comparing the CPU engine (sequential and rayon-batched) against the GPU
pipeline (`GpuPhysicsPipeline`). The point is **not** a leaderboard number — it is
to know, on real hardware, *when* the GPU path is worth using, and to expose the
scale walls that only appear under load.

> Reproduce: `cargo bench -p sim-gpu-benches` (GPU rows need a Metal/Vulkan/DX12
> adapter; without one the CPU rows still run). Quick probe at a single point:
> `cargo run --release -p sim-gpu-benches --example probe -- <substeps> <n_env>`.

## TL;DR

- **The rayon-batched CPU path (`cpu_batch`) is the throughput leader across almost
  the whole range.** On *this* hardware the GPU only pulls ahead at the extreme tail.
- **CPU↔GPU crossover:** for frictionless rollouts the GPU (`gpu_rollout`) reaches
  parity at **n_env ≈ 1024** and edges ahead only at **4096** (1.00 M vs 0.95 M
  env-steps/s). **With contact the GPU never overtakes** the CPU batch in the swept
  range (at 4096: 0.38 M vs 0.64 M).
- **`gpu_rollout` ≫ `gpu_perstep`** everywhere — batching all substeps into one
  submit amortizes the per-step upload/readback that dominates the interactive path.
- **No scale wall** appeared through n_env = 4096 on either fixture (the three walls
  the harness exposed earlier — no-geom panic, ≥100-substep hang, 256 MB buffer
  ceiling — are all fixed upstream; see the GPU shader-conformance notes).
- **Big caveat:** this is an *integrated* Apple GPU sharing memory bandwidth with the
  CPU. The published MJX/Brax/Warp results run on *discrete* A100/H100-class GPUs;
  do not read "CPU wins" as a property of the engine — it is a property of this box.

## Methodology

| | |
|---|---|
| Hardware | Apple M4 Pro — 12 CPU cores, 16-core GPU, Metal 4, unified memory |
| Toolchain | rustc 1.96.0, `--release`, criterion (10 samples, 0.8 s warm-up, 3 s measure) |
| Model | free body (nv = 6) + ground plane + SDF-sphere geom; free joints only (both engines accept it) |
| Step | `STEPS = 100` substeps per measured iteration; dt = 0.002 s ⇒ 0.2 s sim-time/iter |
| Metric | **env-steps / second** = `n_env × STEPS / wall-second` (the metric MJX/Brax report) |
| Batch sweep | n_env ∈ {1, 16, 64, 256, 1024, 4096} |
| Fixtures | `no_contact` (drop from z = 20 → free fall, broadphase finds nothing) · `contact` (start penetrating → full collision + constraint solve every step) |

**The four contenders**

- `cpu_seq` — single-env `Data::step` looped over all envs (naive baseline).
- `cpu_batch` — `BatchSim::step_all`, rayon-parallel across envs (the CPU best).
- `gpu_rollout` — `GpuPhysicsPipeline::step(num_substeps = STEPS)`: all substeps in
  one submit + one readback. The GPU's strength; the right mode for RL-style rollouts.
- `gpu_perstep` — `step(num_substeps = 1)` × STEPS: one upload + readback per step.
  The cost of per-step observation (e.g. a policy that reads state every step).

## Results — env-steps / second (median of 10 samples)

### no_contact (free fall)

| n_env | cpu_seq | cpu_batch | gpu_rollout | gpu_perstep |
|------:|--------:|----------:|------------:|------------:|
| 1     | 79.9 K  | 79.6 K    | 1.35 K      | 0.63 K      |
| 16    | 95.2 K  | 220 K     | 21.4 K      | 10.0 K      |
| 64    | 110 K   | 410 K     | 60.0 K      | 40.8 K      |
| 256   | 116 K   | 648 K     | 323 K       | 80.6 K      |
| 1024  | 116 K   | 861 K     | 860 K       | 198 K       |
| 4096  | 114 K   | 949 K     | **1.00 M**  | 903 K       |

### contact (sustained collision + constraint solve)

| n_env | cpu_seq | cpu_batch | gpu_rollout | gpu_perstep |
|------:|--------:|----------:|------------:|------------:|
| 1     | 79.0 K  | 79.5 K    | 1.82 K      | 0.58 K      |
| 16    | 101 K   | 243 K     | 29.4 K      | 9.29 K      |
| 64    | 99.0 K  | 440 K     | 117 K       | 36.8 K      |
| 256   | 77.4 K  | 651 K     | 394 K       | 127 K       |
| 1024  | 77.2 K  | **713 K** | 462 K       | 228 K       |
| 4096  | 76.3 K  | 636 K     | 378 K       | 362 K       |

## Reading the numbers

- **Per-submit overhead dominates the GPU at small batch.** At n_env = 1 the GPU is
  ~40–60× *slower* than the CPU (59× no_contact, 43× contact): dispatch + readback latency
  swamps 6 DOF of compute. The GPU only becomes sensible once a batch amortizes that
  fixed cost — exactly the regime it is built for.
- **`cpu_batch` scales near-linearly with cores, then saturates** (~0.95 M no_contact,
  ~0.71 M contact) once all 12 cores are busy. `cpu_seq` is flat (~0.11 M) — it never
  uses more than one core, so it is the floor, not a real contender.
- **The crossover is at the tail and frictionless-only.** `gpu_rollout` matches
  `cpu_batch` at n_env ≈ 1024 and passes it at 4096 for `no_contact`. For `contact`,
  the GPU's constraint solve is heavier and it stays behind throughout; both engines
  even *regress* slightly at 4096 contact (memory-bandwidth pressure on a unified-memory
  part).
- **`gpu_rollout` vs `gpu_perstep`** quantifies the cost of reading state every step:
  at n_env = 256 the per-step path is ~3–5× slower. RL pipelines that only need
  end-of-rollout state should use the rollout mode; those needing per-step observations
  pay for it.

## Caveats — do not over-read this

1. **Integrated GPU, unified memory.** The single biggest factor. MJX/Brax/Warp's
   headline throughput is on discrete data-center GPUs (A100/H100) with ~10–40× the
   memory bandwidth and far more compute. A discrete-CUDA run of this same harness
   would very likely move the crossover far to the left. A live head-to-head against
   MJX/Brax/Warp is deferred — it needs a JAX/CUDA environment and is a separate effort;
   their numbers here are *published-reference context*, not a measured comparison.
2. **One small model (6 DOF).** GPU parallelism also scales over *DOF*, not just envs.
   A humanoid (tens–hundreds of DOF) shifts work onto the GPU's strengths; a 6-DOF free
   body is close to the worst case for amortizing GPU overhead. Treat these as a *floor*
   on what the GPU can do, not a ceiling.
3. **f32 GPU vs f64 CPU.** The GPU pipeline runs single-precision; the CPU runs double.
   The path benchmarked here is conformance-verified (GPU↔CPU divergence allowlists are
   empty for this model class), so this is an apples-to-apples *throughput* comparison of
   a correct path — but the engines are not bit-identical.
4. **Criterion sample size is small (10)** to keep the full sweep under a few minutes;
   the medians are stable to a few percent, not benchmark-grade to the last digit.

## What this tells the roadmap

On commodity Apple hardware the well-optimized rayon CPU path is the pragmatic
scale engine today, and the GPU path — now correct and robust — is competitive only
at extreme batch and zero contact. The value of this baseline is the *measurement
infrastructure* and the honest crossover map, not the verdict: the same harness is
what will quantify a discrete-CUDA backend, a larger model, or any future GPU
optimization, and is the throughput oracle for system-ID-at-scale.
