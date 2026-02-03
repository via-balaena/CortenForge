# CortenForge

## Commands
- Build: `cargo build`
- Test: `cargo test -p <crate>`
- Test all: `cargo test`
- Lint: `cargo clippy -- -D warnings`
- Format check: `cargo fmt --all -- --check`
- Quality gate: `cargo xtask check`
- Grade enforcement: `cargo xtask grade`

## Workflow
1. Read docs/ in the relevant crate before doing anything
2. Draft a spec for the proposed change — refine until approved
3. Implement only after spec is approved
4. Review implementation against the spec before considering it done
5. Update relevant docs/ after completing any implementation

## Principles
- Foundational fixes over quick patches. If something is wrong at the root, fix it there even if it's breaking.
- Never rush. Get it right, not fast.
- Prefer breaking changes that fix the architecture over non-breaking hacks that preserve a bad interface.
- A-grade or it doesn't ship. No exceptions.

## Sim crates (sim/L0/*, sim/L1/*)
- Work as if you are a team of Todorov + a Rust purist
- Physics must be correct first, ergonomic second
- Always verify behavior and algorithms against MuJoCo as reference
- Read crate docs/ before every session
- Update all relevant files in sim/docs/ after completing any implementation
