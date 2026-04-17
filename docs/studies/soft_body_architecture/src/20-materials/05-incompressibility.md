# Near-incompressibility

Silicones are nearly incompressible — Poisson's ratio at rest for Ecoflex and Dragon Skin sits at $\nu \approx 0.499$. A soft-body solver that does not represent this correctly produces the volume-loss failure from [Part 1 Ch 02](../10-physical/02-what-goes-wrong/00-volume-loss.md): a compressed sleeve appears to deflate rather than bulge sideways, because the compressive strain energy is not balanced by a volumetric penalty stiff enough to push the material outward into its conserved-volume configuration.

The fix is structurally different from anything linear or hyperelastic alone can do, and is large enough to occupy its own chapter. Naively setting $\lambda \gg \mu$ in a standard Tet4 finite-element discretization does not produce near-incompressible behavior — it produces *volumetric locking*, where the tets are mathematically unable to deform in the specific mode (isochoric bending) that a near-incompressible material actually wants, and the solver returns a spuriously rigid result. The fix is one of four approaches, covered in the sub-chapters.

| Section | Approach | Trade-off |
|---|---|---|
| [Volumetric locking](05-incompressibility/00-locking.md) | What goes wrong, shown concretely — a cantilevered Tet4 beam with $\nu \to 0.5$ that bends 100× less than it should | The setup for the remaining sub-chapters |
| [Mixed u-p formulation](05-incompressibility/01-mixed-up.md) | Add a pressure field as an independent variable; solve for displacement and pressure simultaneously; volumetric term is enforced via the pressure field rather than via the displacement directly | Adds DOFs and a saddle-point solver; most general; handles $\nu$ arbitrarily close to 0.5 |
| [F-bar method](05-incompressibility/02-f-bar.md) | Replace $F$'s volumetric part with an element-averaged version, leaving deviatoric behavior untouched | Cheap Tet4 hack; works for moderate near-incompressibility ($\nu \le 0.49$); degenerates at extreme near-incompressibility |
| [Higher-order elements as alternative](05-incompressibility/03-higher-order.md) | Use Tet10 or Hex8 with sufficient order-of-shape-functions for the incompressibility constraint to be satisfiable | Clean and robust; more expensive per-element; commits to a specific element order |

Two claims Ch 05 rests on:

1. **Incompressibility is not optional.** Silicone's real Poisson ratio at rest is ≈ 0.499, and the [canonical problem](../10-physical/00-canonical.md) involves compression to 10–30% volumetric strain. Setting $\nu = 0.3$ to keep a standard Tet4 solver converging — which is what cheap soft-body pipelines do — gives a fundamentally compressible model that does not match the physical material. The volume-loss failure in Part 1 Ch 02 is the visible consequence of that shortcut.
2. **sim-soft commits to mixed u-p as primary, with F-bar as a fast path.** Mixed u-p handles the full $\nu \to 0.5$ limit and composes cleanly with hyperelastic laws via the pressure field; F-bar is faster but tops out below the silicone regime. `sim-soft` ships both — mixed u-p is the default for silicone materials, F-bar is available as an opt-in for harder materials where $\nu \le 0.49$ and the faster solver matters. Higher-order elements are not the primary fix because Tet4 with mixed u-p is cheaper than Tet10 for comparable accuracy on the canonical problem's strain regime.
