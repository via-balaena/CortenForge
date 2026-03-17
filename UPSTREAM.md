# Upstream Contribution Tracker

Action items for contributing fixes back to our dependencies.
Security advisories themselves are surfaced by `cargo audit` in CI on every PR —
this file tracks what *we* need to do about them.

**Rules:**
- Entries are for our actions (file issue, submit PR, migrate), not for tracking advisories.
- Every entry must have a concrete next action (not just "wait").
- Re-evaluate monthly. Remove once resolved and updated in our lockfile.

---

## Open

### RUSTSEC-2026-0041 — lz4_flex decompression info leak

| Field | Value |
|---|---|
| **Advisory** | [RUSTSEC-2026-0041](https://rustsec.org/advisories/RUSTSEC-2026-0041) |
| **Affected dep** | `lz4_flex 0.7.5` |
| **Our exposure** | `mesh-io → truck-meshalgo 0.4.0 → vtkio 0.6.3 → lz4_flex 0.7.5` |
| **Risk to us** | Low — we only use VTK I/O for mesh import, never decompress untrusted LZ4 data |
| **Fix exists upstream** | Yes — `vtkio 0.7.0-rc1` bumps to `lz4_flex ^0.11` (patched) |
| **Blocked on** | `vtkio 0.7.0` stable release, then `truck-meshalgo` bump to `vtkio ^0.7` |
| **Date found** | 2026-03-17 |

**Action items:**
- [ ] File issue on [elrnv/vtkio](https://github.com/elrnv/vtkio) requesting stable 0.7.0 release, citing RUSTSEC-2026-0041
- [ ] Once vtkio 0.7.0 ships: PR to [ricosjp/truck](https://github.com/ricosjp/truck) bumping `vtkio` dep in `truck-meshalgo`
- [ ] Once truck-meshalgo updates: `cargo update -p truck-meshalgo` here

### bincode 1.x unmaintained (RUSTSEC-2025-0141)

| Field | Value |
|---|---|
| **Advisory** | [RUSTSEC-2025-0141](https://rustsec.org/advisories/RUSTSEC-2025-0141) |
| **Affected dep** | `bincode 1.3.3` |
| **Our exposure** | `sim-mjcf` direct dependency |
| **Risk to us** | Low — using stable serialization features only |
| **Date found** | Pre-existing |

**Action items:**
- [ ] Evaluate migrating `sim-mjcf` from `bincode 1.x` to `bincode 2.x` (breaking API change)

---

## Resolved

_Move entries here once the fix lands in our lockfile._
