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

### paste unmaintained (RUSTSEC-2024-0436)

| Field | Value |
|---|---|
| **Advisory** | [RUSTSEC-2024-0436](https://rustsec.org/advisories/RUSTSEC-2024-0436) |
| **Affected dep** | `paste 1.0.15` |
| **Our exposure** | Transitive: `nalgebra → simba → paste` |
| **Risk to us** | None — proc macro, no runtime risk |
| **Blocked on** | `simba` dropping `paste` dep, or `nalgebra` moving off `simba` |
| **Date found** | 2026-03-23 |

**Action items:**
- [x] Check if `nalgebra 0.34` resolves this — **no**, `simba 0.9` still pulls `paste 1.0.15`
- [ ] File issue on [dimforge/simba](https://github.com/dimforge/simba) about dropping `paste` dep

### proc-macro-error unmaintained (RUSTSEC-2024-0370)

| Field | Value |
|---|---|
| **Advisory** | [RUSTSEC-2024-0370](https://rustsec.org/advisories/RUSTSEC-2024-0370) |
| **Affected dep** | `proc-macro-error 1.0.4` |
| **Our exposure** | Transitive: `mesh-io → truck-stepio → truck-derivers → proc-macro-error` |
| **Risk to us** | None — compile-time only proc macro, no runtime risk |
| **Blocked on** | `truck-derivers` migrating to `proc-macro-error2` or `manyhow` |
| **Date found** | 2026-03-23 |

**Action items:**
- [ ] Check if newer `truck` releases have dropped `proc-macro-error`
- [ ] If not: PR to [ricosjp/truck](https://github.com/ricosjp/truck) migrating `truck-derivers` to `proc-macro-error2`

---

## Resolved

_Move entries here once the fix lands in our lockfile._
