# Friction

Friction is the [missing-friction failure mode](../10-physical/02-what-goes-wrong/03-no-friction.md) from Part 1 Ch 02 given a constitutive treatment. Without friction, the canonical problem has no stable conformity solution — the sleeve slides axially off the probe under any axial load component. Getting friction right means getting stick–slip right, which means writing the friction law as a differentiable energy contribution that resolves the stick regime, the slip regime, and the transition between them without a conditional branch. This chapter's three sub-chapters do that.

| Section | What it covers |
|---|---|
| [Standard Coulomb and its limits](02-friction/00-coulomb.md) | $|t| \le \mu |n|$ with a stick regime ($t$ arbitrary, $\dot u = 0$) and a slip regime ($|t| = \mu |n|$, $\dot u$ parallel to $t$); the stick–slip transition is non-smooth and has no gradient |
| [Smoothed Coulomb](02-friction/01-smoothed.md) | Replace the discontinuous stick regime with a smooth energy $E_\text{fric}(u) = \mu |n| f(\|\dot u\| / \epsilon_v)$ for $f$ a $C^2$ function that is approximately linear for small argument and approximately $\| \cdot \|$ for large. Gradients exist everywhere |
| [Integration with IPC](02-friction/02-ipc-friction.md) | The friction energy is added alongside the barrier energy in the total potential; its coefficient is tied to the local contact normal force, which is itself computed from the barrier. Warm-started within a timestep, lagged across timesteps |

Two claims Ch 02 rests on:

1. **Stick–slip is the hard part.** A solver that does not resolve stick-to-slip transitions correctly will have a sleeve sliding when it should be held, or held when it should be sliding. The [reward function](../10-physical/01-reward.md) depends on the sleeve being held in its conformed configuration; a sleeve that slides under 0.1 N of axial load when it should hold under 1.0 N is not merely inaccurate, it has no stable equilibrium for the optimizer to converge toward. This is why smoothed Coulomb is load-bearing — the smooth transition keeps the Newton solve convergent at every loading point, not just in the pure-stick and pure-slip regimes.
2. **Smoothed Coulomb composes with IPC as another energy term.** The thesis commits to one total potential energy: elasticity + contact + friction + inertia. Each piece is smooth; each contributes a gradient; the Newton loop minimizes the sum. Friction adds one more term and one more warm-start variable (the tangential velocity reference $\dot u_0$) to the solver's state; it does not change the solver's structure. This is the same composition pattern [`Material` compositions](../20-materials/00-trait-hierarchy/01-composition.md) use, applied to the contact layer.
