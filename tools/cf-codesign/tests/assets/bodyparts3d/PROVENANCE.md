# L4 anatomy mesh — for the conduit gate

The anatomy conduit gate (`tests/mesh_body_conduit_anatomy.rs`) routes a conduit around a
real **L4 vertebra** (`FMA13075`), read from `$CF_L4_STL`. The mesh is licensed
(CC BY-SA 2.1 JP) and **never committed** — the deny-by-default `.gitignore` in this
directory keeps a locally-fetched copy out of git.

**Source, pinned commit, per-mesh SHA-256, and the fetch-and-verify recipe are canonical
in** `design/cf-fsu-geometry/BODYPARTS3D.md` **(from the repo root)** — the single
provenance every workspace consumer of these meshes shares. This gate needs only L4.

## Run this gate

```sh
# Fetch + verify FMA13075.stl per the canonical doc above, then point CF_L4_STL at it:
CF_L4_STL=/tmp/bp3d/FMA13075.stl cargo test -p cf-codesign --release \
  --test mesh_body_conduit_anatomy -- --ignored --nocapture
```
