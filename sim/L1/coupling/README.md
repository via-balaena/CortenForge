# cortenforge-sim-coupling

Staggered forward soft↔rigid coupling between sim-soft (FEM) and sim-core (rigid). Exchanges rigid pose (→ soft contact) and the soft contact reaction (→ rigid xfrc) once per lockstep step.

In code, this crate is `sim_coupling`.

This crate is part of [CortenForge](https://github.com/via-balaena/CortenForge), a Rust SDK for mechatronics and simulation. Most applications depend on the [`cortenforge`](https://crates.io/crates/cortenforge) crate instead, which brings in the rest of the SDK.

Licensed under either of the Apache License, Version 2.0, or the MIT license, at your option. Both texts ship with this crate, with a `NOTICE` that carries the disclaimer.
