# Solimp Depth — Impedance Curve and Penetration Depth

**Two heavy balls on a plane with different impedance curves — one sinks deep, one barely penetrates.**

See also: [solref-bounce](../solref-bounce/) | [margin-gap](../margin-gap/)

## What you see

- Green ball (d0=0.9) sits right at the surface — stiff from first contact
- Red ball (d0=0.1) sinks ~19mm into the surface — soft start, stiffens with depth

This is the difference between steel (stiff from contact) and foam (soft start,
progressive resistance).

## Physics

`solimp` controls a sigmoid impedance curve: `[d0, d_width, width, midpoint, power]`

| Parameter | What it controls |
|-----------|-----------------|
| d0 | Impedance at zero violation (0 = no resistance, 1 = full stiffness) |
| d_width | Impedance at full width (endpoint of sigmoid) |
| width | Transition zone in meters |
| midpoint | Midpoint of sigmoid curve |
| power | Steepness of sigmoid |

The impedance `imp` multiplies the position term in the constraint force:
`F = -B * vel - K * imp * depth`

Low d0 means the constraint starts weak and only stiffens as penetration
increases through the transition zone. High d0 means full stiffness from
the first moment of contact.

## Parameters

| Parameter | Stiff ball | Soft ball |
|-----------|-----------|----------|
| d0 | 0.9 | 0.1 |
| d_width | 0.95 | 0.95 |
| width | 0.001 m (1mm) | 0.05 m (50mm) |
| Mass | 2 kg | 2 kg |
| Radius | 0.05 m | 0.05 m |

## Validation

| Check | Expected | Threshold |
|-------|----------|-----------|
| Soft sinks deeper | soft_pen > stiff_pen | PASS at t=5s |
| Stiff minimal penetration | < 5mm | PASS |
| Soft visible penetration | > 2mm | PASS |

## Run

```sh
cargo run -p example-contact-solimp-depth --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
