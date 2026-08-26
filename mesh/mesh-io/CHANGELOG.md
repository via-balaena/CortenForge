# Changelog

All notable changes to mesh-io will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- **License is now `MIT OR Apache-2.0`** (was `Apache-2.0`). The crate inherits
  the workspace declaration (`license.workspace = true`); the dual offer adds the
  MIT option alongside Apache-2.0, at the licensee's choice.

- **Minimum supported Rust version is now 1.92** (was 1.87). The crate inherits
  the workspace-wide declaration, and that floor is set by the locked dependency
  graph as a whole — `wgpu 28.0.0` requires 1.92 — not by anything this crate
  uses, so cargo refuses older toolchains even where this crate alone would have
  built. The 1.87 it replaces was never verified: since 2026-02-17 the weekly
  MSRV job had been trying to install a Rust version that does not exist, so it
  never compiled anything.

### v0.9 candidates

These backlog candidates are gated on a real consumer driving them per
the platform's "examples drive gap-fixes" discipline. Each entry names
the consumer-arrival shape that re-opens the work. The platform-wide
list (sixteen candidates spanning four crates) lives in
[Part 10 of the mesh architecture study](../../docs/studies/mesh_architecture/src/100-roadmap.md);
the entry below is the mesh-io-specific subset.

- **3MF beam writer.** `BeamLatticeData` is already the data model
  (populated by the `strut_cubic` module's `with_beam_export(true)`
  — see mesh book Part 8 Band 6); the writer needs the 3MF Beam
  Lattice Extension format. *Trigger*: 3MF beam-output demand from
  a printer-driver workflow. Effort: ~300-500 LOC.

## [1.0.0] - 2026-05-03

### Added

- First stable release. No functional changes from 0.7.0; the
  semver-major bump indicates API stability commitment per the
  workspace v1.0 milestone (the mesh ecosystem ships its full
  examples-coverage arc at v1.0.0).

## [0.7.0]

Initial release.
