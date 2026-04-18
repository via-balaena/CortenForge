# Build & run instructions

Two distinct artefacts live under this study's roof: the **mdbook** (the book you are reading) and the **`sim-soft` crate** (the software the book specifies). The mdbook builds today. The crate does not exist as code yet — per [Part 11 Ch 03 "the book is the spec"](../110-crate/03-build-order.md), Phase A begins only once the Pass-3-complete book settles its trait surfaces. This appendix names the build path for each.

## Building the mdbook

The mdbook uses the standard [`mdbook`](https://rust-lang.github.io/mdBook/) toolchain plus the [`mdbook-katex`](https://github.com/lzanini/mdbook-katex) preprocessor for equation rendering. Install both:

```text
cargo install mdbook
cargo install mdbook-katex --version 0.10.0-alpha
```

The `0.10.0-alpha` pin is deliberate — stable `0.9.4` breaks against mdbook `0.5`'s TOML contract, and Pass 1 locked the alpha-channel version as the working fixed point (see [Part 6 Ch 05](../60-differentiability/05-diff-meshing.md) scaffolding history for the episode that surfaced the incompatibility).

With both installed, run from the study directory:

```text
cd docs/studies/soft_body_architecture
mdbook serve
```

The default serve port is `3000`; open `http://localhost:3000` in a browser. Edits to files under `src/` trigger automatic rebuild and browser reload. Run `mdbook build` to produce a static `book/` directory without the live server.

The `book.toml` at the study root sets `[preprocessor.katex] after = ["links"]` — the `after = ["links"]` is load-bearing, because KaTeX must run after mdbook has resolved the cross-file link machinery. Reordering it is a Pass-1-trap the study has already stepped in once; don't step in it again.

## The `sim-soft` crate (aspirational)

`sim-soft` does not exist yet. The book is the specification it will be implemented against; Phase A begins once Pass 3 is complete. Once Phase A lands per [Part 11 Ch 03's committed order](../110-crate/03-build-order.md#the-committed-order), the crate will live under `sim/L0/sim-soft/` in the main CortenForge workspace and will build with the existing `cargo xtask` harness that the rest of the workspace uses:

```text
# Expected once Phase A lands:
cargo build -p sim-soft
cargo test  -p sim-soft
cargo xtask grade sim-soft
```

The grade sweep will apply the same seven-criterion rubric the CortenForge workspace's `cargo xtask grade` harness defines for every `sim/L0` crate. `sim-soft` is expected to target an A grade before Phase A is declared complete, consistent with the "A-grade or it doesn't ship" user preference.

At Phase A close, Phase B will layer on the [`element/` and `solver/` modules](../110-crate/03-build-order.md); by Phase D close, the canonical problem will run end-to-end on CPU with exact gradients. The [Part 10 Ch 00 forward-map contract](../100-optimization/00-forward.md) will be the integration surface the [BayesOpt](../100-optimization/02-bayesopt.md) and [active-learning](../100-optimization/04-active-learning.md) layers compile against.

## What cannot be run today

Per the book-is-the-spec commitment, this appendix deliberately does not invent `cargo` invocations that would fail against the absent crate. Benchmarks, regression tests against [MuJoCo flex](../00-context/02-sota/08-mujoco-flex.md), and visual regression against golden images are all Phase-A-onwards work. A reader wanting to experiment with adjacent crates in the CortenForge workspace today (`sim-ml-chassis`, `sim-rl`, `sim-opt`, `sim-bevy`) can work with those directly via the workspace's root `Cargo.toml`; none of them currently depends on `sim-soft`.
