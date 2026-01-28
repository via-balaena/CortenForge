# Future Work

> Low-priority tasks that are out of scope for current development phases.
> These items are tracked here so they don't get lost, but are not blocking any features.

---

## Crate Consolidation

The current crate structure has some redundancy that could be cleaned up:

### sim-constraint

- [ ] Reduce `sim-constraint` to joint definitions only
- Current state: Contains joint types + solver code
- Target state: Joint type definitions only (solver logic consolidated in sim-core)
- Rationale: PGS solver is now in `mujoco_pipeline.rs`, duplicates old solver code

### sim-contact

- [ ] Merge `sim-contact` into `sim-core`
- Current state: Separate crate for contact model
- Target state: Contact types and functions in `sim-core/src/contact.rs`
- Rationale: Contact is tightly coupled to collision; separate crate adds overhead

---

## When to Address

These tasks should be addressed when:
1. Major refactoring is already planned
2. Build times become a concern
3. Crate boundaries cause API friction

Not urgent because:
- Current structure works correctly
- No runtime cost
- Compile times are acceptable
