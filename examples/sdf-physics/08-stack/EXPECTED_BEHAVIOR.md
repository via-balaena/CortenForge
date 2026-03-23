# SDF Physics 08 — Stack: Expected Behavior

## Setup

Three 10mm cubes (5mm half-extents) made of PLA (1250 kg/m³), stacked
vertically on a ground plane. Each cube has a free joint to world.

Initial positions (center z):
- Bottom: 5.5mm (just above ground, 0.5mm gap)
- Middle: 16.0mm (just above bottom cube, 0.5mm gap)
- Top: 26.5mm (just above middle cube, 0.5mm gap)

## What you should see

1. **0–1s**: All three cubes drop slightly as the initial gaps close.
   The bottom cube settles onto the ground first, then the middle onto the
   bottom, and the top onto the middle.

2. **1–3s**: Small oscillations damp out as the solver stabilizes the
   three-body stack. Contact forces propagate through the stack.

3. **3–8s**: The stack is fully settled. All cubes are stationary with
   z-velocities near zero. The stack should look clean and solid — no
   visible jitter, drift, or wobble.

## Expected settled positions

- Bottom cube center: z ≈ 5.0mm (half-extent above ground)
- Middle cube center: z ≈ 15.0mm (3 × half-extent)
- Top cube center: z ≈ 25.0mm (5 × half-extent)

## Contact structure

This example exercises:
- **Ground–cube**: SDF-plane contact (proven in step 04)
- **Cube–cube** (×2): SDF-SDF grid-based multi-contact with face-face
  interface. Unlike sphere-sphere (step 07), cube faces need multiple
  contact points for stability — the grid sampler provides these.

The lever arm stabilization fix in `jacobian.rs` ensures that the wide
contact patches on flat faces don't cause solver instability.

## Pass criteria (automated at t ≥ 5s)

- Each cube z within 2mm of expected height
- Inter-cube gaps ≈ 10mm (2 × half-extent)
- All z-velocities < 1 mm/s
- Contacts active (ncon > 0)

## Failure modes to watch for

- **Tunneling**: A cube passes through another → contact detection failure
- **Jitter**: Cubes vibrate visibly → solver instability or bad friction model
- **Slow collapse**: Stack gradually leans/slides → insufficient friction or
  contact point count
- **Explosion**: Bodies fly apart → contact normal direction error or
  excessive penetration correction
