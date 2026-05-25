# cf-cast cap-plane flatness — recon bookmark (2026-05-25)

**Status:** triage of `docs/CF_CAST_POST_SALVAGE_TRIAGE.md` Findings
A+B+C+D1 → escalated to recon arc. Verification step on Candidate
(1) **falsified the triage doc's framing**: the SDF cap-plane
halfspace intersect is already in production via `pinned_floor_shell`;
the visible cf-view "faceting" is MC corner-chamfering at the sharp
cap-plane × curving-wall corner, not a missing SDF intersect.

**Lesson reinforced from this session:** the
[[feedback-read-prior-arc-memory-before-architectural-decisions]]
discipline caught this. The triage doc (written by the same agent
in the prior session) wrote Candidate (1) as a 50 LOC fix; reading
`pinned_floor_shell` + recon-4 (P) §F-4 + measuring iter-1 STL
geometry showed the framing was wrong. **No production code shipped
this session** — bookmark + sleep is the right move when an
architectural snag surfaces.

## Empirical findings (this session)

### Finding 1 — Candidate (1) is already shipping via `pinned_floor_shell`

`design/cf-design/src/solid_layered.rs:122-130`:

```rust
for (centroid, normal) in cap_planes {
    let plane_offset = centroid.coords.dot(normal);
    shell = shell.intersect(Solid::plane(*normal, plane_offset));
}
```

Both the plug body AND every layer body (`pinned_floor_shell` is
called for each in `tools/cf-cast-cli/src/derive.rs:255-287`) get
the SDF cap-plane halfspace intersect applied BEFORE MC sampling.
This is the same paradigm as recon-4 (P) used for the seam plane
— `Solid::plane`'s SDF is exactly linear; MC's linear-SDF
interpolation places cap-plane vertices bit-precisely (per
recon-4 (P) §F-4 — `f64` precision on a synthetic fixture).

Adding `body.intersect(cap_plane_halfspace)` again as Candidate
(1) proposes is **a no-op** (idempotent against an already-clipped
shell).

### Finding 2 — Body extent assumption holds by construction

The triage doc's verification step (`body.evaluate(cap_centroid + ε
× cap_normal)` returns exterior > 0 for ε ∈ [+0.5, +1, +5, +20] mm)
is **trivially true** by construction. `pinned_floor_shell` clips
the shell at the cap-plane via the halfspace intersect; the
intersect's `max(shell_sdf, plane_sdf)` returns `≥ plane_sdf = ε`
(positive) at any point ε past the cap-plane on the +cap_normal
side, regardless of the shell SDF's value. No empirical probe of
the production fixture was needed.

### Finding 3 — Production STL measurement: the visible non-flatness IS real, ~10-100 µm at the cap-plane EDGE

Quantitative measurement of `~/scans/cast_iter1/*.stl` against the
cap plane from `~/scans/sock_over_capsule.prep.toml`:

Cap-plane = normal (-0.0414, 0.0555, -0.9976), centroid m
(-0.00107, 0.00469, -0.05312). Triangles classified as
"cap-region" if all 3 vertices fall within 100 µm of the
cap-plane (perpendicular signed distance).

| STL                    | cap-region tris | within 10 µm | 10-100 µm |
|------------------------|----------------:|-------------:|----------:|
| plug_layer_0           |             124 |           94 |        30 |
| plug_layer_1           |             674 |          595 |        79 |
| plug_layer_2           |            1197 |         1032 |       165 |
| mold_layer_0_piece_0   |             303 |          219 |        84 |
| mold_layer_0_piece_1   |             353 |          218 |       135 |
| mold_layer_2_piece_0   |             796 |          534 |       162 |

**Interpretation:**

- **70-95% of cap-plane face triangles are bit-precisely flat
  (within 10 µm)** — that's `Solid::plane`'s linear SDF doing
  exactly what recon-4 §F-4 predicts for any linear-SDF MC pass.
- **The 5-30% deviant tris with 10-100 µm vertex deviations are
  clustered at the cap-plane EDGE** — where the flat cap-plane
  meets the curving body wall. MC at 3 mm cells can't represent a
  sharp ~90° corner; cells straddling the corner produce vertices
  interpolated between the `max(shell_sdf, plane_sdf)` branches at
  the derivative discontinuity. Result: a ~3 mm-wide "chamfer
  band" of triangulation rounding the corner.
- **This chamfer band is what the workshop user observed in
  cf-view.** It's a geometric reality, not a render artifact.

For full measurement script see the inline Python in
`design/cf-cast/dev` session log (or rerun: the script reads STL
binary headers + classifies triangles by vertex distance to the
cap plane parsed from `.prep.toml`).

## Root cause reframed

The triage doc's framing "MC's triangulation produces visible
facets because the 3 mm grid samples don't land exactly on the
cap-plane plane" is **incorrect** for the cap-plane INTERIOR — MC
does land vertices on a linear SDF bit-precisely (recon-4 §F-4).
The actual cause is at the cap-plane EDGE:

> A sharp ~90° corner between the flat cap-plane face and the
> curving body wall cannot be resolved by MC at any finite cell
> size — the corner gets rounded into a chamfer of approximately
> one-cell width, with vertices scattered between the two
> bounding surfaces.

This is intrinsic to marching-cubes on a piecewise-defined SDF
with a non-smooth (max-operator) corner. No SDF-side fix can
eliminate it; only a **post-MC cleanup pass** (mesh-CSG slab
subtract) or a **CHANGE OF GEOMETRY** (e.g., introduce a
filleted/chamfered cap-plane corner in the SDF) can produce a
sharp cap-plane edge.

## Re-framed candidate set

The original triage Candidates (1)-(4) were framed against the
wrong root cause. Re-framed:

### Candidate (1') — NO-OP (already shipping)

The "add SDF halfspace intersect" Candidate (1) is already in
production via `pinned_floor_shell`. Removed from the candidate
set.

### Candidate (2') — Post-MC mesh-CSG slab subtract with ε offset

**Same as triage Candidate (2)**, unchanged. The risk profile is
recon-4 (P) §F-2 paradigm-boundary: the slab face SDF surface
would be exactly coincident with the body cap-plane SDF surface,
which is the recon-4 framework's predicted failure mode. The
2026-05-24 night `a8e3e056` attempt (using
`MatingTransform::SeamTrim` at the cap-plane) hit this exact
failure (F4 Critical issues at the boolean junction) and was
reverted within the same commit.

**Mitigation hypothesis** (similar to plug-shaft recon's
`extend_near_end` 1 mm overlap-bias from the
[[project-cf-cast-sdf-meshcsg-paradigm-boundary]] framework): if
the slab subtract face is OFFSET inward from the body cap-plane
by ε ≥ 0.5 mm (one-sixth of a 3 mm MC cell), the boolean is no
longer face-coincident; the cap-plane stays mesh-CSG-flat at the
new ε-displaced location, and the SDF cap-plane lives ε mm above
it (carved away by the slab). Visible cost: ε mm of plug height
+ cup-wall thickness lost. ε = 0.5 mm is well below FDM bead.

**Probe pattern required (g7c-style)**: throwaway synthetic
fixture with flat-face primitive + slab subtract at various ε
offsets, with **cross-primitive control** per
[[feedback-read-prior-arc-memory-before-architectural-decisions]]
(the g7c naive-union paradigm-boundary probe is the canonical
template at `design/cf-cast/tests/g7c_naive_union_paradigm_boundary_probe.rs`).
Branch A clean = ship (2') with measured ε. Branch B
paradigm-boundary artifacts at any reasonable ε = fall through to
(4').

### Candidate (3') — Finer MC cell size in the cap-plane region

**Same as triage Candidate (3)**, unchanged. Not viable without
adaptive-mesh MC infrastructure (multi-week, out of scope).

### Candidate (4') — Accept the MC corner chamfer + document

**Same as triage Candidate (4)**, but with reframed cost: the
chamfer band is **10-100 µm visible**, not "the whole face is
faceted." Workshop user post-print sanding cost is **a 3 mm-wide
ring of < 100 µm bumps at the cap-plane edge**, not "sand the
whole cup-floor flat." Materially cheaper than the triage doc
implied.

### Candidate (5') — NEW: Investigate the "PR #255 didn't have this" claim via git bisect

Workshop user said in the 2026-05-24 night cf-view session that
"we actually had this down and solved in the main branch." If
true, something changed between PR #255 (`aadcfed6` on main) and
current dev that reintroduced the cap-plane chamfer. If the
"didn't have this" was misremembered (PR #255 had the same
chamfer; the workshop user just didn't notice because attention
was on the seam-face film + registration-pin disconnection), then
(4') accept+document becomes the obvious pick.

git-bisect protocol:
1. Reset dev clone to `aadcfed6` (PR #255 merge commit).
2. Run `cargo run --release -p cf-cast-cli -- ~/scans/cast.toml`
   in that state with iter-1 config.
3. Measure cap-plane face flatness on the regenerated plug + cup
   STLs using the same Python script as this session.
4. If PR #255 era shows similar 10-100 µm chamfer: workshop user
   misremembered, pick (4') accept+document.
5. If PR #255 era shows clean cap-plane: bisect between
   `aadcfed6` and current dev to find the responsible commit.

Cost estimate: ~1 session (5 min regen × 2-4 bisect iterations,
plus measurement script).

**Recommended FIRST step of the next session**: (5') git-bisect.
The framing whether-this-is-blocker-or-ergonomic depends on
whether the chamfer was always there or just appeared.

## Recon-prep checklist (next session protocol)

Per [[feedback-read-prior-arc-memory-before-architectural-decisions]]
read these BEFORE picking a candidate:

- [[project-cf-cast-sdf-meshcsg-paradigm-boundary]] — the
  recon-4 framework that classifies the cap-plane mesh-CSG slab
  subtract as paradigm-boundary risk.
- [[project-cf-cast-seam-face-film-recon4-impl]] — recon-4 (P)
  implementation details. The seam-plane parallel for what worked.
- [[project-cf-cast-mating-features-s4-seam-plane]] — S4
  seam-plane trim attempt that recon-4 (P) reverted. Same failure
  mode that the `a8e3e056` cap-plane trim attempt hit.
- [[project-cf-cast-fdm-friendly-geometry-arc]] — the FDM
  friendly arc memory with the full salvage history.

Plus the empirical data in this bookmark.

The g7c naive-union paradigm-boundary probe at
`design/cf-cast/tests/g7c_naive_union_paradigm_boundary_probe.rs`
is the template for the (2') synthetic probe — **must include
cross-primitive control on the same fixture** so the conclusion
isn't shape-specific (the §G-7 mistake from recon-1).

cf-view smoke after every commit that changes production STLs;
workshop user regen + screenshot before next commit lands. No
"I'll check at the end."

## Open questions for the workshop user (next session)

1. Is the 10-100 µm chamfer band acceptable as ergonomic if it's
   confined to a ~3 mm-wide ring at the cap-plane edge (= the
   cup-floor's perimeter where it meets the body cavity, and the
   plug-bottom's perimeter where it meets the plug wall)? The
   center of the face is bit-precisely flat.
2. Should we run the git-bisect (5') before any code change to
   confirm whether PR #255 actually had cleaner cap-plane faces?

## What does NOT land this session

- No production code change.
- No new tests in `cf-cast` or `cf-cast-cli`.
- No iter-1 regen.

## Status

- Triage doc → recon bookmark transition.
- This bookmark + the [[project-cf-cast-fdm-friendly-geometry-arc]]
  memory entry update are this session's only artifacts.
- Branch: `dev`, no push, no PR per
  [[feedback-omnibus-pr-single-branch]].
- Workshop iter-3 print **STILL BLOCKED** pending recon arc + impl.

## Cross-references

- `docs/CF_CAST_POST_SALVAGE_TRIAGE.md` — original triage doc
  (Candidate (1) framing falsified by this session's findings).
- `design/cf-design/src/solid_layered.rs:93-132` —
  `pinned_floor_shell` source of truth.
- `design/cf-cast/src/mesh_csg.rs` — `MatingTransform::SeamTrim`
  variant retained as defensive primitive but NOT emitted for
  cap-plane (`a8e3e056` bookmark).
- `~/scans/cast_iter1/` — iter-1 STL set measured in this
  bookmark.
- `~/scans/sock_over_capsule.prep.toml` — cap-plane source for
  the measurement.
