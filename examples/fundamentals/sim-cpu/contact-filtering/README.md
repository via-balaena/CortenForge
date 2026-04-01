# Contact Filtering — Collision Bitmasks and Exclusion

Contact filtering controls which geometry pairs can collide. Four mechanisms:
bitmask filtering (contype/conaffinity), parent-child auto-exclusion, explicit
body-pair exclusion (`<exclude>`), and explicit geom-pair override (`<pair>`).

## Examples

| Example | Concept | What you see |
|---------|---------|-------------|
| [stress-test](stress-test/) | Headless validation | 12 checks: bitmask AND/OR rule, cross-layer filtering, parent-child exclusion, world body exemption, `<exclude>` suppression, `<pair>` bypass + condim override. |
| [bitmask](bitmask/) | contype/conaffinity bitmask rule | Four spheres with different bitmask values: two rest on the ground, two fall through. |
| [exclude-pairs](exclude-pairs/) | `<exclude>` body-pair suppression | Three boxes; excluded pair passes through each other, non-excluded pair collides normally. |
| [ghost-layers](ghost-layers/) | Collision layers via bitmasks | Solid and ghost layers: solids stack, ghosts rest on solids but pass through each other. |

## Key ideas

- **Bitmask rule:** Two geoms collide if `(g1.contype & g2.conaffinity) != 0 ||
  (g2.contype & g1.conaffinity) != 0`. It's an OR — one matching side is enough.
- **Defaults:** `contype=1`, `conaffinity=1` — everything collides with everything.
- **Parent-child auto-exclusion:** Bodies connected by a joint don't self-collide.
  World body (body 0) is exempt — ground planes always collide.
- **`<exclude body1="a" body2="b"/>`:** Suppresses all contact between two bodies.
  Bidirectional — body order doesn't matter.
- **`<pair geom1="a" geom2="b" .../>`:** Bypasses bitmask filtering and applies
  custom solver parameters (condim, friction, solref, solimp, margin, gap).
