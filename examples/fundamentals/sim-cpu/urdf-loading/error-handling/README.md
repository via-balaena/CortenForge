# Error Handling

Feeds several invalid URDFs to the parser and validates that each produces
the correct error variant — not a panic. The crate should fail gracefully
with actionable, human-readable error messages that include enough context
to diagnose the problem.

## What it tests

The `sim-urdf` error types:

| Error variant | Trigger |
|---------------|---------|
| `MissingElement` | No `<robot>` element in XML |
| `UnknownJointType` | `type="invalid_type"` on a joint |
| `UndefinedLink` | Joint references a link that doesn't exist |
| `DuplicateLink` | Two links with the same name |
| `XmlParse` | Malformed XML (unclosed tags, etc.) |

Each error message includes the relevant names (link name, joint name,
element name) so the user knows exactly where the problem is.

## Checks

| # | Check | Tolerance |
|---|-------|-----------|
| 1 | Missing `<robot>` → error (not panic) | exact |
| 2 | Unknown joint type → `UnknownJointType` variant | exact |
| 3 | Undefined link → `UndefinedLink` variant | exact |
| 4 | Duplicate link → `DuplicateLink` variant | exact |
| 5 | Malformed XML → error (not panic) | exact |
| 6 | Empty string → error (not panic) | exact |
| 7 | Error message includes context (link + joint names) | exact |

## Run

```
cargo run -p example-urdf-error-handling --release
```
