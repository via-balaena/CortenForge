# Self-contact

A sleeve folded on itself, a compliant gripper closing until its fingers touch, a catheter sheath buckling and contacting its own interior surface — all of these are self-contact. The IPC barrier does not distinguish self from non-self; the energy is the same, the barrier form is the same, the smoothness guarantees are the same. What changes is the combinatorics of generating the contact pairs without the cost exploding.

| Section | What it covers |
|---|---|
| [Bounding volume hierarchies](03-self-contact/00-bvh.md) | Axis-aligned bounding box tree over the deformed surface mesh; rebuilt or refit each timestep; pruning ratio typically 100×–10000× at the canonical problem's resolution |
| [Proximity pair generation](03-self-contact/01-proximity.md) | Point-triangle and edge-edge pair enumeration inside the BVH-identified candidate set; excluding same-triangle and adjacent-triangle pairs to avoid spurious zero-distance reports |

Two claims Ch 03 rests on:

1. **Self-contact is not a special case at the barrier level.** The IPC contact energy is $\kappa \sum_i b(d_i, \hat d)$ over all primitive pairs within distance $\hat d$ of each other; a self-pair (two triangles from the same surface) contributes exactly the same energy shape as a cross-pair (sleeve surface vs. probe surface). Nothing in the solver, Newton loop, or autograd pipeline branches on "is this pair self-contact." This is what keeps the [energy-based formulation](01-ipc-internals/03-energy.md) clean: self-contact is an input to the pair generator, not a separate code path through the solver.
2. **The BVH is what makes self-contact tractable.** Cross-contact (sleeve-to-probe) is $O(N \cdot M)$ pairs without acceleration; self-contact is $O(N^2)$ worst-case, which for a 30k-vertex sleeve is ~10⁹ pairs — infeasible per timestep. A well-constructed BVH reduces the pair count to $O(N \cdot K)$ where $K$ is the average number of proximate primitives per primitive (typically 5–50 for deformed contact), making the per-step cost linear in $N$ instead of quadratic. [Part 4 Ch 05's real-time chapter](05-real-time.md) treats BVH construction as one of the three main engineering levers for real-time IPC; without it, even the canonical problem's single-layer sleeve becomes infeasible under self-contact.
