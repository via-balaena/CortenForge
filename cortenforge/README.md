# cortenforge

A Rust SDK for mechatronics and simulation: geometry, parametric design, mesh
processing, rigid- and soft-body physics, and reinforcement learning.

This crate is a facade. It re-exports the crates listed below and adds no code
of its own, so an application depends on this one crate and reaches the SDK
through it. It is headless: no GUI, Bevy or GPU crate comes with it.

## Features

All three are on by default.

- `sim`: simulation, as `cortenforge::sim`.
- `mesh`: mesh processing, as `cortenforge::mesh`.
- `fabrication`: design and scan preparation, as `cortenforge::cf_design`,
  `cortenforge::cf_cap_planes`, `cortenforge::cf_device_types` and
  `cortenforge::cf_scan_prep_core`.

`sim` and `fabrication` also bring in `cortenforge::cf_geometry` and
`cortenforge::cf_spatial`. To build only part of the SDK:

```sh
cargo add cortenforge --no-default-features --features mesh
```

With the default features, every path above is there:

```rust
use cortenforge::{cf_cap_planes, cf_design, cf_device_types, cf_scan_prep_core};
use cortenforge::{cf_geometry, cf_spatial, mesh, sim};
```

## Disclaimer

CortenForge is general-purpose research and engineering software, provided
**as is** under MIT or Apache-2.0 at your option, **without warranty of any
kind**. **It is not a medical device** and makes no medical, therapeutic, or
health claims. You use it, and make and use anything created with it,
**entirely at your own risk**: you alone are responsible for deciding whether a
design is fit for what you intend to do with it, and for the materials,
fabrication, testing, and operation of anything you build. What you build with
it can involve serious hazards: hydrogen and other compressed or flammable
gases, pressure vessels, high-voltage systems, moving machinery, vehicles that
are not certified for road use, and materials that contact the body. The full
text is in the `NOTICE` file that ships with this crate, and in
[DISCLAIMER.md](https://github.com/via-balaena/CortenForge/blob/main/DISCLAIMER.md).

## License

Licensed under either of the Apache License, Version 2.0, or the MIT license,
at your option. Both texts ship with this crate.
