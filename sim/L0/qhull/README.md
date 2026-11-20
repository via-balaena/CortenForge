# cortenforge-sim-qhull

Qhull's 3-d convex hull, ported to Rust as MuJoCo 3.5.0 runs it, giving MuJoCo's mesh graph

In code, this crate is `sim_qhull`.

This crate is part of [CortenForge](https://github.com/via-balaena/CortenForge), a Rust SDK for mechatronics and simulation.

It is a port of [Qhull](http://www.qhull.org): the path `qhull Qt` takes over 3-d points, as MuJoCo 3.5.0 calls it to make a mesh's convex hull graph (`mjCMesh::MakeGraph`), to give MuJoCo's graph vertex for vertex and triangle for triangle; an input that reaches a routine it does not hold is refused, naming the routine (`Error::Unported`). The original Qhull source may be obtained from www.qhull.org (and from github.com/qhull/qhull; this port follows commit `62ccc56a`, the one MuJoCo 3.5.0 builds).

Qhull, Copyright (c) 1993-2020 C.B. Barber, Arlington, MA, and The National Science and Technology Research Center for Computation and Visualization of Geometric Structures (The Geometry Center), University of Minnesota. The Qhull portions are licensed under Qhull's licence, `COPYING.txt`, which ships with this crate. They were modified in November 2026 by Via Balaena (the CortenForge authors): translated to Rust and cut to the 3-d path MuJoCo 3.5.0 runs, so that a mesh's convex hull graph is MuJoCo's.

The part that reads MuJoCo's graph from qhull's result is ported from [MuJoCo](https://github.com/google-deepmind/mujoco) (Copyright 2021 DeepMind Technologies Limited) and licensed under the Apache License, Version 2.0.

There is no warranty: Qhull is provided "as is" (`COPYING.txt`, condition 5), and so is this port. CortenForge is not affiliated with or endorsed by C.B. Barber, The Geometry Center, the Qhull project or Google DeepMind.
