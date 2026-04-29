# mesh-shell SDF path engine fix — commit 11.5 plan

> Working memo for the dedicated session(s) fixing `generate_shell_sdf` + `generate_rim_for_sdf_shell` before commit 12 (`shell-generation-high-quality`) can resume. Slots in as commit 11.5 of the mesh-ecosystem-to-par PR. Branch: `feature/examples-soft-body`. Pre-flight on commit 12 revealed `.high_quality()` produces a non-printable shell on every input shape tested. Delete this file after commit 12 ships and the arc closes (per `feedback_code_speaks`).

## Why this exists

Pre-flight on commit 12 (`shell-generation-high-quality`) revealed `ShellBuilder::new(&inner).high_quality().build()` produces a non-printable shell on both the open-topped 10mm box (commit 11's input) AND a closed 10mm cube. Empirical signature on both inputs:

```text
is_printable() == false
is_watertight  == false  (~75k boundary edges)
is_inside_out  == true
signed_volume  ≈ -3,500   (negative, confirming inside-out)
outer_vertex_count ≈ 75k  (vertex-soup; vertex_count == 3 × face_count)
```

Plus, on the open-box variant, one additional NonManifold edge + one DegenerateTriangle + 8 nonsense rim faces from `generate_rim_for_sdf_shell` collapsing on a vertex-soup outer.

User decision (option B from the proposal): stop, fix the engine, then resume commit 12 — but in this PR via dedicated session(s) rather than a separate fix-only PR. Phase A commit count grows from 14 to 14+N where N = engine-fix commits.

## Bugs confirmed (recon session 2026-04-29)

### Bug 1 — vertex-soup outer not welded

**Location**: `mesh/mesh-shell/src/shell/generate.rs:255-296` (`generate_shell_sdf`).

**Algorithm-level confirmation**: `mesh-offset/src/marching_cubes.rs:131-140` unconditionally pushes 3 fresh vertices per triangle:

```rust
let base_idx = mesh.vertices.len() as u32;
mesh.vertices.push(edge_vertices[e0]);
mesh.vertices.push(edge_vertices[e1]);
mesh.vertices.push(edge_vertices[e2]);
mesh.faces.push([base_idx, base_idx + 1, base_idx + 2]);
```

Every MC triangle is a vertex-island. `generate_shell_sdf` concatenates this soup directly into the shell at `generate.rs:280-296`. No mitigation exists in the SDF path. (The Normal path produces no soup because it offsets each input vertex 1:1.)

This was already documented in commit 9's outcome ("Marching cubes vertex non-sharing — exact `vertex_count == 3 × face_count`"; `mesh-offset/src/marching_cubes.rs::process_cell` lines 131-137). What's new in commit 11.5 is realizing that mesh-shell never compensated for the soup before stitching the outer into the shell.

### Bug 2 — MC inside-out winding inherited unflipped

**Location**: `mesh/mesh-shell/src/shell/generate.rs:289-296`.

**Anchored by**: mesh-architecture book commit-9 outcome ("Platform discovery #1 — `mesh-offset` MC produces inside-out winding") and re-confirmed commit 10 ("Inside-out quirk is sign-agnostic"). `signed_volume < 0` and `validate_mesh.is_inside_out == true` for any positive-distance offset.

**In `generate_shell_sdf`**: outer faces are written with original winding plus index offset; no flip:

```rust
// Add outer faces (keep original winding, offset indices)
for face in &outer_mesh.faces {
    shell.faces.push([
        face[0] + inner_count,
        face[1] + inner_count,
        face[2] + inner_count,
    ]);
}
```

By contrast, inner faces are deliberately winding-reversed (`[face[0], face[2], face[1]]` at line 286) so they point INTO the cavity (correct for cavity-facing). Outer faces should be reversed too — but for a different reason: to flip the MC inside-out so they point OUTWARD. Bug: the reversal isn't applied.

Combined with bug 1 (no welding), the divergence-theorem integral for `signed_volume` resolves negative — every face's contribution has the wrong sign.

### Bug 3 — rim collapse on open input

**Location**: `mesh/mesh-shell/src/shell/rim.rs:85-133` (`generate_rim_for_sdf_shell`).

**Two-layer issue**:

**Layer 1 (soup-induced)**: when outer is unwelded, `find_boundary_loops` on the soup returns one mini-loop per MC triangle. Each face has 3 edges, each edge appears in exactly one face (vertex-soup → no shared edges), `find_boundary_edges` (rim.rs:138-175) returns ALL edges as boundary, `find_boundary_loops` (rim.rs:178-246) traces 3-vertex loops one-per-MC-triangle. The "best match by centroid distance" inner→outer picks one arbitrary 3-vertex outer loop. `stitch_boundary_loops(4-vert inner, 3-vert outer)` enters the linear-interp branch (rim.rs:298-315) and emits 8 nonsense faces with a NonManifold edge + DegenerateTriangle.

**Layer 2 (topological, post-weld)**: even with welding fixed, the SDF level set of an open mesh produces a **closed wrap-around** with curved caps over the open boundary — NOT a 1-sided offset surface.

Reasoning for the 10mm open box (5-sided, top open) at +2.5mm offset: the level set wraps over the top opening as a quarter-cylinder of radius 2.5mm around each top edge, joined by quarter-sphere octants at the four top corners. The wrap is a closed manifold with no boundary loops. So `find_boundary_loops(welded_outer)` returns `[]` for any open input. `generate_rim_for_sdf_shell` early-returns at `rim.rs:93-95`. The combined shell has the inner's open boundary edges (4 in the open-box case) unstitched → not watertight.

**This means `WallGenerationMethod::Sdf` cannot produce bowl-semantics output for open inputs without additional lid-finding + cut-and-stitch machinery (~200-300 LOC of new geometry, out of scope for this fix).**

## Contracts re-confirmed in mesh-repair

`weld_vertices(&mut mesh, epsilon)` (`mesh-repair/src/repair.rs:299-381`):
- Spatial-hash with `cell_size = 2 × epsilon`, 3³ neighborhood scan.
- Returns merge count.
- Remaps face indices via internal vertex_remap table (line 370-374).
- Removes degenerate faces (`i0==i1` etc.) post-merge.
- Does **not** compact the vertex array — must follow with `remove_unreferenced_vertices` to shrink.
- O(N) build + near-O(N) merge for uniformly distributed input.
- mesh-repair already a direct dep of mesh-shell; no new deps needed.

`fix_winding_order` (`mesh-repair/src/winding.rs:66-172`):
- BFS from arbitrary start face per component; establishes **consistency**, not orientation.
- On a globally inside-out mesh, all faces end up consistent but inside-out.
- Cannot replace per-face flip for bug 2.

Per commit-9 platform truth: **`face.swap(1, 2)` per-face flip is the canonical remediation** for soup-mesh / MC-output winding correction (adjacency-free, position-preserving, idempotent under double application).

## Existing test suite — no broken-behavior pins

No `tests/` dir; all tests inline `#[cfg(test)] mod tests`.

| File | Tests | Path |
|---|---|---|
| `generate.rs::tests` | 6 | All use `.fast()` (Normal method) or test config introspection — none exercise SDF generation |
| `rim.rs::tests` | 6 | All on Normal-rim path; `generate_rim_for_sdf_shell` is untested |
| `validation.rs::tests` | 5 | Standalone validation; engine-agnostic |
| `builder.rs::tests` | 5 | Config-only |

**No test pins broken SDF behavior.** Coverage gap: zero existing tests assert `.high_quality()` produces a printable shell on any input. The fix introduces both a bug fix AND a coverage gap fix.

`builder.rs::test_high_quality_preset` asserts `builder.wall_method == WallGenerationMethod::Sdf` at builder-config time; the bug 3 fallback happens at generator runtime so this builder test is unaffected.

## Grade baseline

`cargo xtask grade mesh-shell --skip-coverage` → A on all 7 automated criteria pre-fix. Goal: keep at A through the fix.

## Fix plan

### Engine changes (in `generate.rs::generate_shell_sdf` only — no public API changes)

**Step 1 (bug 1 + bug 2)** — after `offset_mesh()` succeeds:

```rust
let mut outer_mesh = ...;  // existing offset_mesh call
let weld_eps = params.sdf_voxel_size_mm * 1e-3;
let merged = weld_vertices(&mut outer_mesh, weld_eps);
let _orphans = remove_unreferenced_vertices(&mut outer_mesh);
debug!("Welded {merged} duplicate verts in outer; compacted");
for face in &mut outer_mesh.faces {
    face.swap(1, 2);  // flip MC inside-out per commits 9+10 platform truth
}
```

Tolerance rationale: `voxel_size × 1e-3` keeps cell_size sub-voxel (3e-4 mm at 0.3 mm voxel), so we merge MC's exactly-coincident soup duplicates without merging legitimately-distinct vertices. MC interpolation is deterministic, so coincident duplicates are bit-identical — `1e-9` would suffice in theory, but `voxel_size × 1e-3` is more defensive against floating-point drift.

Per-face flip (not `fix_winding_order` + signed-volume probe) per commit-9 platform truth and the canonical pattern. Unconditional because mesh-offset 0.7.x emits inside-out on every input we've tested across commits 9+10.

**Step 2 (bug 3)** — after step 1, before concatenation:

```rust
use mesh_repair::MeshAdjacency;
let inner_has_boundary = MeshAdjacency::build(&inner_mesh.faces).boundary_edge_count() > 0;
let outer_has_boundary = MeshAdjacency::build(&outer_mesh.faces).boundary_edge_count() > 0;
if inner_has_boundary && !outer_has_boundary {
    warn!(
        "SDF wall generation: open input wraps to closed level set; \
         falling back to Normal method (constant-thickness SDF cannot lid an open mesh)"
    );
    return generate_shell_normal(inner_mesh, params);
}
```

Rationale: fallback over error because (a) `generate_shell_sdf` already has a fallback pattern at `generate.rs:257-261` for `offset_mesh` failures — consistent with that; (b) user-facing "asked for high-quality, got something printable + a warning" beats "got an error and had to switch APIs"; (c) mathematically, SDF on open input produces a closed wrap that can't match the bowl semantics of Normal — fallback makes the semantic match the user's intent.

`ShellGenerationResult.wall_method` ends up `Normal` after fallback (consistent with existing fallback path); consumers can detect the fallback via `result.wall_method`.

### Doc updates

Update `WallGenerationMethod::Sdf` doc string (`generate.rs:32-39`) to add: "Open inputs (with boundary edges) automatically fall back to Normal method since the SDF level set forms a closed wrap-around rather than a one-sided offset."

### New tests (inline in `generate.rs::tests`)

1. **`sdf_shell_on_closed_cube_is_printable`** — primary regression. Build closed cube (`create_open_box` + 2 top faces). Run `ShellBuilder::new(&closed).high_quality().build()`. Assert: `validation.is_watertight`, `validation.is_manifold`, `validation.is_printable()`. Critical positive test for the fix.
2. **`sdf_shell_outer_winding_outward_after_flip`** — same closed cube; assert resulting shell has `validate_mesh(&shell).is_inside_out == false` (signed_volume > 0). Anchors bug 2 fix.
3. **`sdf_shell_outer_is_welded_no_soup`** — same closed cube; assert `shell.vertices.len() < 4 × shell.faces.len()` (vertex-soup would give vertex_count ≈ 3 × outer_face_count + inner_count, much larger than welded). Anchors bug 1 fix.
4. **`sdf_shell_on_open_box_falls_back_to_normal`** — `create_open_box`, `.high_quality().build()`, assert `result.wall_method == Normal` (fallback fired) and `validation.is_printable() == true`. Anchors bug 3 fallback. Pre-fix code returned `wall_method == Sdf` with non-printable shell.
5. **`sdf_shell_open_input_outer_count_matches_inner`** — open box; the fallback uses Normal method which produces `outer_vertex_count == inner.vertices.len()`. Sanity check the fallback truly takes the Normal path.

~80 LOC of tests, all inline. Engine changes ~30 LOC total. Total estimated diff ~110 LOC.

### Suggested commit segmentation

Two commits, both on `feature/examples-soft-body`:

- **commit 11.5.1** — `fix(mesh-shell): weld and flip outer in SDF path` (bugs 1+2 together; tightly coupled, single atomic change). Includes tests 1, 2, 3.
- **commit 11.5.2** — `fix(mesh-shell): SDF path falls back to Normal for open input` (bug 3). Includes tests 4, 5 + doc-string update.

Per-commit verification: `cargo clippy -p mesh-shell --release --all-targets -- -D warnings` + `cargo test -p mesh-shell --release` + `cargo fmt -p mesh-shell -- --check`. Final `cargo xtask grade mesh-shell --skip-coverage` once both land.

### Blast-radius assessment for downstream consumers

- **Closed-input callers of `.high_quality()`**: previously got non-printable shell (silent garbage); now get printable shell. Strict improvement.
- **Open-input callers of `.high_quality()`**: previously got non-printable shell; now get printable shell via Normal fallback + `tracing::warn`. Semantic shift: consumers asking for "constant-thickness SDF level set" get Normal-method per-vertex offset instead. Detectable via `result.wall_method`. Since the previous output was unusable, no consumer can be relying on it.
- **Public API surface**: unchanged — `ShellBuilder`, `ShellBuildResult`, `ShellParams`, `WallGenerationMethod`, `ShellGenerationResult`, `validate_shell` keep their signatures.
- **Future commit 12** (`shell-generation-high-quality` example): can use a closed input from the start (closed cube or torus) to anchor the SDF code path's true behavior, OR use an open input and demonstrate the fallback as load-bearing pedagogy. User to decide for the example commit.

## Open questions for user (pre-implementation)

1. **Bug 3 fix shape**: fallback (recommended) vs error vs best-effort-with-warning? Recommendation is fallback, anchored above.
2. **Commit segmentation**: 2 commits as proposed, or single all-in-one? Recommendation: 2 commits for review-by-review separation.
3. **Tolerance for `weld_eps`**: `voxel_size × 1e-3` (defensive sub-voxel) vs `1e-9` (tight, MC-determinism only)? Recommendation: defensive.
4. **Commit 12 example input**: closed cube/torus as SDF anchor, or open box + fallback-as-pedagogy? Decide later when commit 12 resumes.

## Stress-test status

- [ ] Reproduction: closed cube, `.high_quality()`, observe `is_printable() == false` + `is_inside_out == true` + `vertex_count == 3 × outer_face_count + inner_count`
- [ ] Reproduction: open box, `.high_quality()`, observe additional 1 NonManifold edge + 1 DegenerateTriangle + 8 nonsense rim faces from `generate_rim_for_sdf_shell` collapse
- [ ] Spike: weld outer alone (no flip), observe what changes
- [ ] Spike: per-face flip alone (no weld), observe what changes
- [ ] Spike: weld + flip on closed cube → does it become printable? (proves bugs 1+2 fix is sufficient for closed-input case)
- [ ] Spike: weld + flip on open box → still 4 unstitched inner-boundary edges? (proves bug 3 needs separate handling)
