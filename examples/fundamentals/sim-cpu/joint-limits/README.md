# Joint Limits — Limit Constraints with Solver Tuning

Joint limits constrain the range of motion. When a joint reaches its limit, the
solver generates a one-sided constraint. The stiffness and damping of the limit
response are tuned via `solref` and `solimp` — the same solver parameters used
for contacts.

## Examples

| Example | Concept | What you see |
|---------|---------|-------------|
| [stress-test](stress-test/) | Headless validation | 12 checks: hinge/slide/ball activation, sensor readback, solref scaling, solimp width, motor vs limit, locked joint, force vs penetration, cone symmetry. |
| [hinge-limits](hinge-limits/) | Solref tuning comparison | Three pendulums (stiff/default/soft) released beyond limit — different bounce and penetration behavior. |
| [slide-limits](slide-limits/) | Motor vs limit constraint | Box on rail pushed by motor into limit — limit force balances motor force. |
| [ball-cone](ball-cone/) | Ball joint cone limit | Rod on ball joint with 30-degree cone — symmetric limit regardless of azimuth. |

## Key ideas

- **`limited="true"` + `range="lo hi"`:** Enables limit constraints on a joint.
  Range is in degrees (if `compiler angle="degree"`) for hinge/ball, meters for slide.
- **One-sided constraint:** Only activates when the joint exceeds its range.
  No force when inside the range (`JointLimitFrc == 0`).
- **`solreflimit="timeconst dampratio"`:** Controls stiffness/damping of the limit
  response. Shorter timeconst = stiffer. Default: `0.02 1.0`.
- **`solimplimit="d0 dwidth width midpoint power"`:** Controls the impedance
  sigmoid — how the constraint force scales with penetration depth.
- **Ball joint cone:** `range="0 30"` restricts total deflection to a 30-degree
  cone. The cone is rotationally symmetric.
- **`<jointlimitfrc joint="name"/>`:** Sensor that reports constraint force at
  the limit. Zero when interior, positive when at or beyond the limit.
