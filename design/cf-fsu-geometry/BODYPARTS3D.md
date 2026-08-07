# BodyParts3D anatomy meshes — canonical provenance

This is the **single source of truth** for the lumbar Functional Spinal Unit (FSU)
meshes the workspace loads through [`cf_fsu_geometry::load_from_env`](src/lib.rs). Those
meshes are a **licensed asset — CC BY-SA 2.1 JP — and are never committed**: every
consumer is `#[ignore]`d (or a manual tool) and reads each mesh path from an environment
variable. Fetch them to a temp path and point the env vars at them.

Consumers across the workspace key off these same three env vars — the FSU ladder tests
(`sim/L1/coupling/tests/rung7_fsu_validation.rs`, `rung6c_disc_geometry.rs`,
`fsu_coupled_contact.rs`), `sim-bevy-soft`'s render examples, `cf-design-tests`,
`cf-routing-tests`, `cf-spine-studio`, and the `cf-codesign` anatomy conduit gate — so
they all resolve here.

## Source

The meshes come from **BodyParts3D / Anatomography** (DBCLS, Japan), via the GitHub clone
that names each file by its Foundational Model of Anatomy (FMA) ID:

- Repository: <https://github.com/Kevin-Mattheus-Moerman/BodyParts3D>, **pinned to commit
  `f0eeb6e843380cfe6b83797cf8c3e1af74de5e61`** (`main` is a mutable ref — pinning gives
  reproducibility, since a changed mesh would silently shift the ladder/gate numbers,
  with supply-chain integrity as a bonus).
- Original database: <https://dbarchive.biosciencedbc.jp/en/bodyparts3d/download.html>

| part        | FMA ID   | path in the repo                           | env var       | bytes  | SHA-256 |
|-------------|----------|--------------------------------------------|---------------|--------|---------|
| L4 vertebra | FMA13075 | `assets/BodyParts3D_data/stl/FMA13075.stl` | `CF_L4_STL`   | 771384 | `3464b94d03d42e43bf5ba01a741b7677bec8113a8359eb4a2caec960c6cf341c` |
| L5 vertebra | FMA13076 | `assets/BodyParts3D_data/stl/FMA13076.stl` | `CF_L5_STL`   | 864184 | `244f54f1388b1cb02014a6de9c1b32ef40d398967bc8e17ed56b46c637fe11c1` |
| L4–L5 disc  | FMA16036 | `assets/BodyParts3D_data/stl/FMA16036.stl` | `CF_DISC_STL` | 724584 | `865f96b3f3bfed6536808edadd46c7ff742c6ee6e93eb7e8705863643cec7a94` |

Most consumers need only a subset: the `cf-codesign` conduit gate uses only L4; the disc
rungs use only the disc; `cf-spine-studio` uses L4 + L5 and lofts the disc itself. The
FSU validation (`rung7_fsu_validation.rs`) uses all three.

## Fetch + verify

```sh
commit=f0eeb6e843380cfe6b83797cf8c3e1af74de5e61
base=https://raw.githubusercontent.com/Kevin-Mattheus-Moerman/BodyParts3D/$commit/assets/BodyParts3D_data/stl

# Pull the triad to a temp dir (binary STLs). Do NOT commit them.
mkdir -p /tmp/bp3d
for fma in FMA13075 FMA13076 FMA16036; do
  curl -sSL -o "/tmp/bp3d/$fma.stl" "$base/$fma.stl"
done

# Verify integrity before use — abort if any bytes differ from the pin.
( cd /tmp/bp3d && shasum -a 256 -c - <<'SUMS'
3464b94d03d42e43bf5ba01a741b7677bec8113a8359eb4a2caec960c6cf341c  FMA13075.stl
244f54f1388b1cb02014a6de9c1b32ef40d398967bc8e17ed56b46c637fe11c1  FMA13076.stl
865f96b3f3bfed6536808edadd46c7ff742c6ee6e93eb7e8705863643cec7a94  FMA16036.stl
SUMS
)

# Point the env vars at them, then run any consumer, e.g. the FSU validation:
export CF_L4_STL=/tmp/bp3d/FMA13075.stl
export CF_L5_STL=/tmp/bp3d/FMA13076.stl
export CF_DISC_STL=/tmp/bp3d/FMA16036.stl
cargo test -p sim-coupling --release --test rung7_fsu_validation -- --ignored --nocapture
```

## Which gates need them

Do not maintain that list by hand — ask for it. `cargo xtask licensed-gates`
derives the surface from the source tree (every `#[test]` whose `#[ignore]`
reason names one of the three variables) and prints it grouped by crate;
`--run` executes it, and `--only <crate>` narrows to one:

```sh
cargo xtask licensed-gates                 # what exists, and which meshes each needs
cargo xtask licensed-gates --run           # run all of it (needs the paths exported)
cargo xtask licensed-gates --only cf-codesign --run
```

A run demands only the meshes its selected gates actually name, so an L4-only
crate runs with L4 alone.

**Why this is derived rather than listed.** These gates cannot run in CI and
never will — the meshes are licensed and uncommittable — so the only thing
standing between them and silent rot is somebody running them deliberately.
A hand-kept list is the wrong instrument for that: twice in the rung-β arc an
inventory was recorded that was narrower than its name, and both times the
gates it omitted had already gone red.

**Run it before merging a change to production code these gates exercise** —
geometry, meshing, the solver path, or the FSU model — since nothing else will.
A test-only or docs-only change to those crates does not need it, because the
gates cannot observe one; scope with `--only <crate>` when the blast radius is
one crate. The full surface is ~2 h 21 m, so applying the rule where it cannot
matter is how it stops being followed where it does.

## Coordinates

The meshes are in **native millimetres**, positioned in the whole-atlas coordinate frame
(L4 sits near `z ≈ 970 mm`). [`cf_fsu_geometry::load`] keeps native coordinates
deliberately, so sibling vertebrae stay in their shared anatomical frame and stack. A
rescale to SI metres is a soft-FEM-solver concern (the keystone coupling operates in
metres); pure-geometry consumers work in native mm.

## License + citation

The meshes are **CC BY-SA 2.1 JP** (Creative Commons Attribution-ShareAlike 2.1 Japan).
Attribution:

> Mitsuhashi N, Fujieda K, Tamura T, Kawamoto S, Takagi T, Okubo K.
> *BodyParts3D: 3D structure database for anatomical concepts.*
> Nucleic Acids Research, 2009, 37(Database issue):D782–D785.
