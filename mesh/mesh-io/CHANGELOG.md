# Changelog

All notable changes to mesh-io will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### v0.9 candidates

These backlog candidates are gated on a real consumer driving them per
the platform's "examples drive gap-fixes" discipline. Each entry names
the consumer-arrival shape that re-opens the work. The platform-wide
list (sixteen candidates spanning four crates) lives in
[Part 10 of the mesh architecture study](../../docs/studies/mesh_architecture/src/100-roadmap.md);
the entry below is the mesh-io-specific subset.

- **3MF beam writer.** `BeamLatticeData` is already the data model
  (populated by `mesh-lattice-strut-cubic`'s `with_beam_export(true)`
  — see mesh book Part 8 Band 6); the writer needs the 3MF Beam
  Lattice Extension format. *Trigger*: 3MF beam-output demand from
  a printer-driver workflow. Effort: ~300-500 LOC.

## [0.7.0]

Initial release.
