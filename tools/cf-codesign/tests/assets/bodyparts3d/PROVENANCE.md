# BodyParts3D anatomy meshes — provenance

The anatomy conduit gate (`tests/mesh_body_conduit_anatomy.rs`) routes a conduit
around a **real lumbar vertebra** instead of an analytic primitive. That mesh is a
licensed asset — **CC BY-SA 2.1 JP** — so it is **never committed**: the gate is
`#[ignore]`d and reads the mesh path from `$CF_L4_STL`, exactly like the workspace's
FSU ladder tests (`sim/L1/coupling/tests/rung7_fsu_validation.rs` et al.).

## Source

The lumbar Functional Spinal Unit meshes come from **BodyParts3D / Anatomography**
(DBCLS, Japan), via the GitHub clone that names each file by its Foundational Model of
Anatomy (FMA) ID:

- Repository: <https://github.com/Kevin-Mattheus-Moerman/BodyParts3D> (pinned to commit
  `f0eeb6e8` in the fetch below — `main` is a mutable ref)
- Original database: <https://dbarchive.biosciencedbc.jp/en/bodyparts3d/download.html>

| part        | FMA ID   | path in the repo                             | env var       |
|-------------|----------|----------------------------------------------|---------------|
| L4 vertebra | FMA13075 | `assets/BodyParts3D_data/stl/FMA13075.stl`   | `CF_L4_STL`   |
| L5 vertebra | FMA13076 | `assets/BodyParts3D_data/stl/FMA13076.stl`   | `CF_L5_STL`   |
| L4–L5 disc  | FMA16036 | `assets/BodyParts3D_data/stl/FMA16036.stl`   | `CF_DISC_STL` |

The conduit gate uses **only L4**. The L5 / disc rows document the shared FSU triad the
rest of the workspace's `#[ignore]`d ladder tests key off the same way.

## Fetch + run

```sh
# Pinned to a specific commit and verified by checksum — these exact bytes or fail.
# `main` is mutable; pinning gives reproducibility (a changed mesh would silently shift
# the gate's numbers) with supply-chain integrity as a bonus.
commit=f0eeb6e843380cfe6b83797cf8c3e1af74de5e61
sha256=3464b94d03d42e43bf5ba01a741b7677bec8113a8359eb4a2caec960c6cf341c
url=https://raw.githubusercontent.com/Kevin-Mattheus-Moerman/BodyParts3D/$commit/assets/BodyParts3D_data/stl/FMA13075.stl

# Pull L4 to a temp path (binary STL, 771384 bytes ≈ 753 KB). Do NOT commit it.
curl -sSL -o /tmp/FMA13075.stl "$url"

# Verify integrity before use — abort if the bytes differ from the pin.
echo "$sha256  /tmp/FMA13075.stl" | shasum -a 256 -c -

# Run the env-gated anatomy gate (release — the signed-distance grid build is heavy).
CF_L4_STL=/tmp/FMA13075.stl cargo test -p cf-codesign --release \
  --test mesh_body_conduit_anatomy -- --ignored --nocapture
```

## Coordinates

The meshes are in **native millimetres**, positioned in the whole-atlas coordinate
frame (L4 sits near `z ≈ 970 mm`). The gate works in this native mm frame unchanged —
**not** because the objective is scale-free (it is not: the target's finite-difference
step and the optimiser's `grad_tol` are absolute), but because *every* constant in the
gate (clearance margin, grid cell, padding, the FD step, and the optimiser tolerances)
is consistently in millimetres. A rescale to SI metres is a soft-FEM-solver concern;
this pure-geometry gate does not use it.

## License + citation

The meshes are **CC BY-SA 2.1 JP** (Creative Commons Attribution-ShareAlike 2.1 Japan).
Attribution:

> Mitsuhashi N, Fujieda K, Tamura T, Kawamoto S, Takagi T, Okubo K.
> *BodyParts3D: 3D structure database for anatomical concepts.*
> Nucleic Acids Research, 2009, 37(Database issue):D782–D785.
