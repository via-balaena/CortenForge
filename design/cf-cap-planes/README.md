# cortenforge-cap-planes

Cap-plane parsing for cleaned scans: reads a .prep.toml [caps] block and strips cap-polygon faces from the scan, so consumers can build cap-stripped (dome-wall-only) signed-distance fields

In code, this crate is `cf_cap_planes`.

This crate is part of [CortenForge](https://github.com/via-balaena/CortenForge), a Rust SDK for mechatronics and simulation. Most applications depend on the [`cortenforge`](https://crates.io/crates/cortenforge) crate instead, which brings in the rest of the SDK.

Licensed under either of the Apache License, Version 2.0, or the MIT license, at your option. Both texts ship with this crate, with a `NOTICE` that carries the disclaimer.
