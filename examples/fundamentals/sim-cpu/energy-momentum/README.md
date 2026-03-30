# Energy-Momentum — Conservation Laws

Five examples demonstrating the most fundamental correctness properties
of a physics engine: energy and momentum conservation. Each example
introduces exactly one new concept, building on the previous.

## Concept Ladder

| # | Example | What it demonstrates |
|---|---------|---------------------|
| 1 | [free-flight](free-flight/) | Conservation with zero external forces — KE, linear momentum, and angular momentum all constant to machine precision |
| 2 | [pendulum-energy](pendulum-energy/) | Energy exchange between potential and kinetic forms under gravity, with total energy conserved |
| 3 | [elastic-bounce](elastic-bounce/) | Energy conservation across a collision event — ball bounces to 97% of drop height |
| 4 | [damped-decay](damped-decay/) | Energy dissipation from damping — amplitude shrinks to zero, energy never increases |
| 5 | [stress-test](stress-test/) | Headless validation of all the above (12 checks) |

## Key APIs

- `<flag energy="enable"/>` — activates energy computation in the pipeline
- `data.energy_kinetic` — kinetic energy (0.5 * qvel^T * M * qvel)
- `data.energy_potential` — gravitational + joint spring + tendon spring PE
- `data.total_energy()` — KE + PE
- `SubtreeAngMom` sensor — total angular momentum about system COM

## Lessons Learned

- **PE reference is arbitrary.** Gravitational PE depends on the coordinate
  origin, so total energy can be negative. Use energy *differences* (drift,
  PE drop) for meaningful comparisons, not ratios of total energy.

- **Contact spring PE is untracked.** During contact, `energy_potential`
  doesn't include the elastic energy stored in the contact spring. Total
  energy appears to dip during contact and recovers after separation.
  Measure conservation at the bounce apex (free flight), not during contact.

- **Angular velocity is not angular momentum.** For asymmetric bodies,
  angular velocity precesses in the world frame (torque-free precession).
  Angular *momentum* is the conserved quantity, not angular velocity.
