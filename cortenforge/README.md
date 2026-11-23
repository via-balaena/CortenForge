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

Portions of some of the crates this one brings in are ported from
[MuJoCo](https://github.com/google-deepmind/mujoco) and
[OpenSim](https://github.com/opensim-org/opensim-core), whose source code is
licensed under the Apache License, Version 2.0, sim-mjcf's OBJ reader from
[tinyobjloader](https://github.com/tinyobjloader/tinyobjloader), under the MIT
license (its ear clipping's point-in-polygon test, W. Randolph Franklin's
PNPOLY, under its own license as well), sim-core's distance to a mesh from
[TriangleMeshDistance](https://github.com/InteractiveComputerGraphics/TriangleMeshDistance),
under the MIT license, and sim-core's sort and red-black tree from LLVM's
[libc++](https://github.com/llvm/llvm-project), under the Apache License v2.0
with LLVM Exceptions; those portions stay under their licenses whichever license
you take for the rest. The `NOTICE` that ships with this crate names those
crates in its MUJOCO, OPENSIM, TINYOBJLOADER, TRIANGLEMESHDISTANCE and LIBCXX
sections and carries the projects' notices. CortenForge is not affiliated with
or endorsed by Google DeepMind, Stanford University, the OpenSim project, Syoyo
Fujita, the tinyobjloader project, W. Randolph Franklin, José Antonio Fernández
Fernández, the TriangleMeshDistance project, the LLVM Foundation or the LLVM
Project.

One crate this one brings in is not under the choice above:
cortenforge-sim-qhull, a port of [Qhull](http://www.qhull.org), is licensed
under Qhull's license (its `COPYING.txt`), for the part ported from MuJoCo the
Apache License, Version 2.0, and for its sort, ported from Apple's Libc, the
3-clause BSD license. The `NOTICE` that ships with this crate carries their
copyright notices in its QHULL and QSORT sections. CortenForge is not
affiliated with or endorsed by C.B. Barber, The Geometry Center, the Qhull
project, the University of California or Apple.
